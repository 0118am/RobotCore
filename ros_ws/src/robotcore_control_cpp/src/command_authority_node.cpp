#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotcore_interfaces/msg/body_state.hpp"
#include "robotcore_interfaces/msg/control_authority_status.hpp"
#include "robotcore_interfaces/msg/safety_event.hpp"
#include "robotcore_interfaces/msg/thruster_command.hpp"
#include "robotcore_interfaces/msg/trajectory_target.hpp"
#include "robotcore_interfaces/srv/set_control_authority.hpp"

using namespace std::chrono_literals;

namespace robotcore_control_cpp
{
class CommandAuthorityNode final : public rclcpp::Node
{
  using ThrusterCommand = robotcore_interfaces::msg::ThrusterCommand;
  using BodyState = robotcore_interfaces::msg::BodyState;
  using TrajectoryTarget = robotcore_interfaces::msg::TrajectoryTarget;
  using SafetyEvent = robotcore_interfaces::msg::SafetyEvent;
  using AuthorityStatus = robotcore_interfaces::msg::ControlAuthorityStatus;
  using SetAuthority = robotcore_interfaces::srv::SetControlAuthority;
  using SteadyClock = std::chrono::steady_clock;

public:
  CommandAuthorityNode()
  : Node("command_authority"), last_tick_(SteadyClock::now())
  {
    evaluation_rate_hz_ = std::max(1.0, declare_parameter("evaluation_rate_hz", 100.0));
    publish_rate_hz_ = std::max(1.0, declare_parameter("publish_rate_hz", 50.0));
    status_rate_hz_ = std::max(0.1, declare_parameter("status_publish_rate_hz", 10.0));
    candidate_timeout_s_ = declare_parameter("candidate_timeout_s", 0.10);
    state_timeout_s_ = declare_parameter("state_timeout_s", 0.15);
    target_timeout_s_ = declare_parameter("target_timeout_s", 0.15);
    safety_timeout_s_ = declare_parameter("safety_heartbeat_timeout_s", 0.25);
    automatic_limit_ = declare_parameter("automatic_command_limit", 0.15);
    automatic_slew_ = declare_parameter("automatic_slew_rate_per_s", 0.5);
    allow_rl_ = declare_parameter("allow_rl_hardware", false);
    pool_configured_ = declare_parameter("pool_bounds_configured", false);
    pool_min_ = vector3_parameter("pool_min_xyz");
    pool_max_ = vector3_parameter("pool_max_xyz");

    command_pub_ = create_publisher<ThrusterCommand>(
      "/control/thruster_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    status_pub_ = create_publisher<AuthorityStatus>(
      "/control/authority/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    for (std::size_t i = 0; i < sources_.size(); ++i) {
      const auto source = sources_[i];
      candidate_subs_[i] = create_subscription<ThrusterCommand>(
        "/control/candidates/" + source, rclcpp::QoS(rclcpp::KeepLast(1)),
        [this, source](ThrusterCommand::SharedPtr message) {
          candidates_[source] = TimedCandidate{std::move(message), SteadyClock::now()};
        });
    }
    body_sub_ = create_subscription<BodyState>(
      "/robot/body_state", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](BodyState::SharedPtr message) {
        if (message->state_valid) {absolute_localization_seen_ = true;}
        body_ = TimedBody{std::move(message), SteadyClock::now()};
      });
    target_sub_ = create_subscription<TrajectoryTarget>(
      "/runtime/trajectory_target", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](TrajectoryTarget::SharedPtr message) {
        target_ = TimedTarget{std::move(message), SteadyClock::now()};
      });
    safety_sub_ = create_subscription<SafetyEvent>(
      "/safety/events", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const SafetyEvent::SharedPtr message) {on_safety(*message);});
    authority_service_ = create_service<SetAuthority>(
      "/control/authority/set",
      [this](const std::shared_ptr<SetAuthority::Request> request,
        std::shared_ptr<SetAuthority::Response> response) {
        on_set_authority(*request, *response);
      });

    evaluation_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / evaluation_rate_hz_),
      std::bind(&CommandAuthorityNode::evaluate, this));
    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() {publish_command(output_enabled_);});
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_rate_hz_),
      [this]() {publish_status(SteadyClock::now());});
    RCLCPP_INFO(
      get_logger(), "C++ command authority started (evaluation %.1f Hz, command %.1f Hz)",
      evaluation_rate_hz_, publish_rate_hz_);
  }

private:
  struct TimedCandidate {ThrusterCommand::SharedPtr message; SteadyClock::time_point stamp;};
  struct TimedBody {BodyState::SharedPtr message; SteadyClock::time_point stamp;};
  struct TimedTarget {TrajectoryTarget::SharedPtr message; SteadyClock::time_point stamp;};

  std::array<double, 3> vector3_parameter(const std::string & name)
  {
    const auto values = declare_parameter<std::vector<double>>(name, {0.0, 0.0, 0.0});
    if (values.size() != 3U) {
      throw std::invalid_argument(name + " must contain exactly three values");
    }
    return {values[0], values[1], values[2]};
  }

  static double age_s(
    const std::optional<SteadyClock::time_point> & stamp, SteadyClock::time_point now)
  {
    if (!stamp) {return std::numeric_limits<double>::infinity();}
    return std::chrono::duration<double>(now - *stamp).count();
  }

  static std::string lower(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return value;
  }

  static bool supported(const std::string & source)
  {
    return source == "manual" || source == "pid" || source == "rl";
  }

  std::string candidate_integrity_failure(
    const std::string & source, SteadyClock::time_point now) const
  {
    const auto item = candidates_.find(source);
    if (item == candidates_.end() ||
      std::chrono::duration<double>(now - item->second.stamp).count() > candidate_timeout_s_)
    {
      return source + " candidate is missing or stale";
    }
    const auto expected = expected_producers_.find(source);
    if (expected == expected_producers_.end() || item->second.message->source != expected->second) {
      return "unexpected candidate producer: " + item->second.message->source;
    }
    for (const auto value : item->second.message->normalized) {
      if (!std::isfinite(value)) {return "candidate command is not eight finite values";}
    }
    return {};
  }

  std::string candidate_failure(const std::string & source, SteadyClock::time_point now) const
  {
    auto reason = candidate_integrity_failure(source, now);
    if (!reason.empty()) {return reason;}
    if (!candidates_.at(source).message->enable) {return source + " candidate is not ready";}
    return {};
  }

  std::string common_failure(SteadyClock::time_point now) const
  {
    if (abort_active_) {return "safety abort is active";}
    if (fault_latched_) {return "authority fault is latched: " + fault_code_;}
    if (!safety_stamp_ || age_s(safety_stamp_, now) > safety_timeout_s_) {
      return "safety monitor heartbeat is missing";
    }
    return {};
  }

  bool inside_pool() const
  {
    if (!body_) {return false;}
    const auto & p = body_->message->pose.position;
    const std::array<double, 3> point{p.x, p.y, p.z};
    for (std::size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(point[i]) || !(pool_min_[i] < pool_max_[i]) ||
        point[i] < pool_min_[i] || point[i] > pool_max_[i]) {return false;}
    }
    return true;
  }

  std::string automatic_failure(SteadyClock::time_point now) const
  {
    if (!pool_configured_) {return "pool bounds are not configured";}
    if (!body_ || std::chrono::duration<double>(now - body_->stamp).count() > state_timeout_s_) {
      return "body state is missing or stale";
    }
    if (!target_ || std::chrono::duration<double>(now - target_->stamp).count() > target_timeout_s_) {
      return "trajectory target is missing or stale";
    }
    if (!absolute_localization_seen_) {return "absolute localization has not been observed";}
    const auto & body = *body_->message;
    if (!body.linear_velocity_valid) {return "linear velocity is invalid";}
    if (!body.state_valid && !body.position_estimated) {return "localization is invalid";}
    if (body.position_estimated && body.localization_source.rfind("ZED VIO", 0) != 0U) {
      return "estimated localization source is not allowed";
    }
    if (!target_->message->valid) {return "trajectory target is invalid";}
    if (!inside_pool()) {return "vehicle is outside configured pool bounds";}
    return {};
  }

  std::string prearm_failure(SteadyClock::time_point now) const
  {
    auto reason = common_failure(now);
    if (!reason.empty()) {return reason;}
    reason = candidate_failure(selected_source_, now);
    if (!reason.empty()) {return reason;}
    return selected_source_ == "manual" ? std::string{} : automatic_failure(now);
  }

  std::string manual_idle_reason(SteadyClock::time_point now) const
  {
    if (selected_source_ != "manual") {return {};}
    const auto item = candidates_.find("manual");
    if (item == candidates_.end() ||
      std::chrono::duration<double>(now - item->second.stamp).count() > candidate_timeout_s_)
    {
      return "manual candidate is missing or stale";
    }
    if (!item->second.message->enable) {return "manual candidate is not ready";}
    return {};
  }

  void trip(const std::string & code, const std::string & message, bool publish = true)
  {
    armed_ = false;
    fault_latched_ = true;
    fault_code_ = code;
    message_ = message;
    output_.fill(0.0);
    output_enabled_ = false;
    if (publish) {publish_command(false);}
  }

  void on_safety(const SafetyEvent & event)
  {
    abort_active_ = event.abort_active;
    safety_stamp_ = SteadyClock::now();
    if (abort_active_ && armed_) {
      trip(event.code.empty() ? "ABORT_ACTIVE" : event.code,
        event.message.empty() ? "safety abort" : event.message);
    }
  }

  void fill_response(SetAuthority::Response & response, bool accepted, const std::string & text)
  {
    response.accepted = accepted;
    response.selected_source = selected_source_;
    response.armed = armed_;
    response.fault_latched = fault_latched_;
    response.message = text;
  }

  void on_set_authority(const SetAuthority::Request & request, SetAuthority::Response & response)
  {
    auto requested = lower(request.source.empty() ? selected_source_ : request.source);
    if (!supported(requested)) {
      message_ = "unsupported source: " + requested;
      fill_response(response, false, message_); return;
    }
    if (armed_ && requested != selected_source_) {
      message_ = "control source can change only while disarmed";
      fill_response(response, false, message_); return;
    }
    if (requested == "rl" && !allow_rl_) {
      message_ = "RL hardware authority is disabled";
      fill_response(response, false, message_); return;
    }
    if (request.clear_fault) {
      if (abort_active_) {
        message_ = "clear /safety/abort before clearing authority fault";
        fill_response(response, false, message_); return;
      }
      fault_latched_ = false; fault_code_.clear(); message_ = "fault cleared";
      if (request.arm == armed_ && requested == selected_source_) {
        fill_response(response, true, message_); return;
      }
    }
    if (!armed_) {selected_source_ = requested;}
    if (!request.arm) {
      armed_ = false; output_enabled_ = false; output_.fill(0.0); message_ = "disarmed";
      publish_command(false);
      fill_response(response, true, message_); return;
    }
    const auto reason = prearm_failure(SteadyClock::now());
    if (!reason.empty()) {message_ = reason; fill_response(response, false, reason); return;}
    if (!armed_) {++arm_generation_;}
    armed_ = true; message_ = "armed " + selected_source_;
    fill_response(response, true, message_);
  }

  void evaluate()
  {
    const auto steady_now = SteadyClock::now();
    const auto dt = std::clamp(
      std::chrono::duration<double>(steady_now - last_tick_).count(), 0.0, 0.1);
    last_tick_ = steady_now;

    const auto manual_reason = candidate_integrity_failure("manual", steady_now);
    const auto manual_item = candidates_.find("manual");
    const bool manual_active = manual_reason.empty() && manual_item->second.message->enable;
    const bool manual_override = armed_ && selected_source_ != "manual" && manual_active;
    active_source_ = manual_override ? "manual" : selected_source_;
    bool output_allowed = armed_;
    std::string idle_reason;

    if (output_allowed) {
      const auto reason = manual_override ? common_failure(steady_now) : prearm_failure(steady_now);
      if (!reason.empty()) {
        idle_reason = manual_idle_reason(steady_now);
        if (reason == idle_reason && !idle_reason.empty()) {
          output_.fill(0.0); message_ = "armed manual; neutral: " + idle_reason;
        } else {
          trip("CONTROL_INPUT_INVALID", reason, false); output_allowed = false;
        }
      }
    }
    if (output_allowed && idle_reason.empty()) {
      const auto & values = candidates_.at(active_source_).message->normalized;
      for (std::size_t i = 0; i < output_.size(); ++i) {
        double value = values[i];
        if (active_source_ == "pid" || active_source_ == "rl") {
          const auto limit = std::abs(automatic_limit_);
          value = std::clamp(value, -limit, limit);
          const auto delta = std::abs(automatic_slew_) * dt;
          value = std::clamp(value, output_[i] - delta, output_[i] + delta);
        }
        output_[i] = std::clamp(value, -1.0, 1.0);
      }
      message_ = manual_override ?
        "armed " + selected_source_ + "; manual LB override" : "armed " + selected_source_;
    } else {output_.fill(0.0);}

    output_enabled_ = output_allowed && idle_reason.empty();
  }

  void publish_command(bool enable)
  {
    ThrusterCommand command;
    command.header.stamp = now(); command.header.frame_id = "base_link";
    for (std::size_t i = 0; i < output_.size(); ++i) {
      command.normalized[i] = static_cast<float>(output_[i]);
    }
    command.enable = enable; command.armed = armed_;
    command.arm_generation = arm_generation_;
    command.source = "command_authority:" + active_source_;
    command_pub_->publish(command);
  }

  void publish_status(SteadyClock::time_point steady_now)
  {
    AuthorityStatus status;
    status.header.stamp = now(); status.selected_source = selected_source_;
    status.armed = armed_; status.arm_generation = arm_generation_;
    status.abort_active = abort_active_; status.fault_latched = fault_latched_;
    status.fault_code = fault_code_; status.message = message_;
    const auto selected = candidates_.find(selected_source_);
    status.candidate_age_s = age_s(
      selected == candidates_.end() ? std::optional<SteadyClock::time_point>{} :
      std::optional<SteadyClock::time_point>{selected->second.stamp}, steady_now);
    status.body_state_age_s = age_s(
      body_ ? std::optional<SteadyClock::time_point>{body_->stamp} : std::nullopt, steady_now);
    status.target_age_s = age_s(
      target_ ? std::optional<SteadyClock::time_point>{target_->stamp} : std::nullopt, steady_now);
    status.command_limit = automatic_limit_; status.command_slew_rate = automatic_slew_;
    status.localization_source = body_ ? body_->message->localization_source : "";
    status.pool_bounds_configured = pool_configured_;
    status_pub_->publish(status);
  }

  const std::array<std::string, 3> sources_{"manual", "pid", "rl"};
  const std::unordered_map<std::string, std::string> expected_producers_{
    {"manual", "web_operator"}, {"pid", "pid_controller"}, {"rl", "rl_action_adapter"}};
  std::unordered_map<std::string, TimedCandidate> candidates_;
  std::optional<TimedBody> body_;
  std::optional<TimedTarget> target_;
  std::optional<SteadyClock::time_point> safety_stamp_;
  SteadyClock::time_point last_tick_;
  std::array<double, 8> output_{};
  std::array<double, 3> pool_min_{}, pool_max_{};
  std::string selected_source_{"manual"}, active_source_{"manual"};
  std::string fault_code_, message_{"disarmed"};
  bool armed_{false}, abort_active_{false}, fault_latched_{false};
  bool output_enabled_{false};
  bool absolute_localization_seen_{false}, allow_rl_{false}, pool_configured_{false};
  std::uint64_t arm_generation_{0U};
  double evaluation_rate_hz_{}, publish_rate_hz_{}, status_rate_hz_{};
  double candidate_timeout_s_{}, state_timeout_s_{}, target_timeout_s_{}, safety_timeout_s_{};
  double automatic_limit_{}, automatic_slew_{};

  rclcpp::Publisher<ThrusterCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<AuthorityStatus>::SharedPtr status_pub_;
  std::array<rclcpp::Subscription<ThrusterCommand>::SharedPtr, 3> candidate_subs_;
  rclcpp::Subscription<BodyState>::SharedPtr body_sub_;
  rclcpp::Subscription<TrajectoryTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<SafetyEvent>::SharedPtr safety_sub_;
  rclcpp::Service<SetAuthority>::SharedPtr authority_service_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_, command_timer_, status_timer_;
};
}  // namespace robotcore_control_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotcore_control_cpp::CommandAuthorityNode>());
  rclcpp::shutdown();
  return 0;
}
