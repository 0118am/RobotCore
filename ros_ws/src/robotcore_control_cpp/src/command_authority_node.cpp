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
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "robotcore_interfaces/msg/body_state.hpp"
#include "robotcore_interfaces/msg/control_authority_status.hpp"
#include "robotcore_interfaces/msg/safety_event.hpp"
#include "robotcore_interfaces/msg/thruster_command.hpp"
#include "robotcore_interfaces/msg/trajectory_target.hpp"
#include "robotcore_interfaces/srv/set_control_authority.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

namespace robotcore_control_cpp
{
constexpr double kActionPwmSpanUs = 250.0;
constexpr double kMaximumPwmLimitUs = 250.0;
constexpr double kMaximumAction = 1.0;

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
  : Node("command_authority")
  {
    evaluation_rate_hz_ = std::max(1.0, declare_parameter("evaluation_rate_hz", 100.0));
    publish_rate_hz_ = std::max(1.0, declare_parameter("publish_rate_hz", 50.0));
    status_rate_hz_ = std::max(0.1, declare_parameter("status_publish_rate_hz", 10.0));
    source_command_timeout_s_ = declare_parameter("source_command_timeout_s", 0.10);
    state_timeout_s_ = declare_parameter("state_timeout_s", 0.15);
    target_timeout_s_ = declare_parameter("target_timeout_s", 0.15);
    safety_timeout_s_ = declare_parameter("safety_heartbeat_timeout_s", 0.25);
    const auto pwm_limit_us = declare_parameter("pwm_limit_us", kMaximumPwmLimitUs);
    action_limit_ = std::clamp(
      std::abs(pwm_limit_us) / kActionPwmSpanUs, 0.0, kMaximumAction);
    allow_rl_ = declare_parameter("allow_rl_hardware", true);

    command_pub_ = create_publisher<ThrusterCommand>(
      "/control/thruster_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    status_pub_ = create_publisher<AuthorityStatus>(
      "/control/authority/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    manual_command_sub_ = create_source_subscription(
      "/control/manual/thruster_cmd", "manual", "web_operator");
    pid_command_sub_ = create_source_subscription(
      "/control/pid/thruster_cmd", "pid", "pid_controller");
    rl_action_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/policy/body/action", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr message) {
        on_rl_action(*message);
      });
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
  struct TimedSourceCommand {ThrusterCommand::SharedPtr message; SteadyClock::time_point stamp;};
  struct TimedBody {BodyState::SharedPtr message; SteadyClock::time_point stamp;};
  struct TimedTarget {TrajectoryTarget::SharedPtr message; SteadyClock::time_point stamp;};

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

  rclcpp::Subscription<ThrusterCommand>::SharedPtr create_source_subscription(
    const std::string & topic, const std::string & source, const std::string & producer)
  {
    return create_subscription<ThrusterCommand>(
      topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this, source, producer](ThrusterCommand::SharedPtr message) {
        if (message->source != producer) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "ignored %s command from unexpected producer '%s'",
            source.c_str(), message->source.c_str());
          return;
        }
        source_commands_[source] =
          TimedSourceCommand{std::move(message), SteadyClock::now()};
      });
  }

  void on_rl_action(const std_msgs::msg::Float32MultiArray & action)
  {
    if (action.data.size() != 8U ||
      !std::all_of(action.data.begin(), action.data.end(),
        [](float value) {
          return std::isfinite(value) && value >= -1.0F && value <= 1.0F;
        }))
    {
      source_commands_.erase("rl");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignored RL action: expected eight finite values in [-1, 1]");
      return;
    }

    auto command = std::make_shared<ThrusterCommand>();
    command->header.stamp = now();
    command->header.frame_id = "base_link";
    for (std::size_t channel = 0; channel < 8U; ++channel) {
      command->action[channel] = action.data[channel];
    }
    command->enable = true;
    command->source = "t60_policy";
    source_commands_["rl"] = TimedSourceCommand{std::move(command), SteadyClock::now()};
  }

  std::string source_command_integrity_failure(
    const std::string & source, SteadyClock::time_point now) const
  {
    const auto item = source_commands_.find(source);
    if (item == source_commands_.end() ||
      std::chrono::duration<double>(now - item->second.stamp).count() >
      source_command_timeout_s_)
    {
      return source + " command is missing or stale";
    }
    const auto expected = expected_producers_.find(source);
    if (expected == expected_producers_.end() || item->second.message->source != expected->second) {
      return "unexpected command producer: " + item->second.message->source;
    }
    for (const auto value : item->second.message->action) {
      if (!std::isfinite(value) || value < -1.0F || value > 1.0F) {
        return "source action is not eight finite values in [-1, 1]";
      }
    }
    return {};
  }

  std::string source_command_failure(
    const std::string & source, SteadyClock::time_point now) const
  {
    auto reason = source_command_integrity_failure(source, now);
    if (!reason.empty()) {return reason;}
    if (!source_commands_.at(source).message->enable) {
      return source + " command is not ready";
    }
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

  std::string automatic_failure(SteadyClock::time_point now) const
  {
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
    if (!target_->message->valid) {return "trajectory target is invalid";}
    return {};
  }

  std::string prearm_failure(SteadyClock::time_point now) const
  {
    auto reason = common_failure(now);
    if (!reason.empty()) {return reason;}
    reason = source_command_failure(selected_source_, now);
    if (!reason.empty()) {return reason;}
    return selected_source_ == "manual" ? std::string{} : automatic_failure(now);
  }

  std::string manual_idle_reason(SteadyClock::time_point now) const
  {
    if (selected_source_ != "manual") {return {};}
    const auto item = source_commands_.find("manual");
    if (item == source_commands_.end() ||
      std::chrono::duration<double>(now - item->second.stamp).count() >
      source_command_timeout_s_)
    {
      return "manual command is missing or stale";
    }
    if (!item->second.message->enable) {return "manual command is not ready";}
    return {};
  }

  bool altitude_hold_target_active(SteadyClock::time_point now) const
  {
    return target_ &&
      std::chrono::duration<double>(now - target_->stamp).count() <= target_timeout_s_ &&
      target_->message->valid && lower(target_->message->control_mode) == "altitude_hold";
  }

  static std::array<double, 8> altitude_manual_surge_yaw(
    const ThrusterCommand & manual)
  {
    const auto & values = manual.action;
    const double surge = 0.25 * (-values[4] - values[5] + values[6] + values[7]);
    const double yaw = 0.25 * (-values[4] + values[5] + values[6] - values[7]);
    return {0.0, 0.0, 0.0, 0.0,
      -surge - yaw, -surge + yaw, surge + yaw, surge - yaw};
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
    response.pwm_limit_us = action_limit_ * kActionPwmSpanUs;
    response.message = text;
  }

  void on_set_authority(const SetAuthority::Request & request, SetAuthority::Response & response)
  {
    if (request.update_pwm_limit) {
      if (!request.source.empty() || request.arm || request.clear_fault) {
        fill_response(response, false, "PWM limit update cannot change authority state");
        return;
      }
      if (!std::isfinite(request.pwm_limit_us) ||
        request.pwm_limit_us < 0.0 || request.pwm_limit_us > kMaximumPwmLimitUs)
      {
        fill_response(response, false, "PWM limit must be finite and within 0..250 us");
        return;
      }
      action_limit_ = request.pwm_limit_us / kActionPwmSpanUs;
      message_ = "PWM limit set to " + std::to_string(request.pwm_limit_us) + " us";
      fill_response(response, true, message_);
      publish_status(SteadyClock::now());
      return;
    }
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

    const auto manual_reason = source_command_integrity_failure("manual", steady_now);
    const auto manual_item = source_commands_.find("manual");
    const bool manual_active = manual_reason.empty() && manual_item->second.message->enable;
    const bool hybrid_altitude_hold = armed_ && selected_source_ == "pid" &&
      manual_active && altitude_hold_target_active(steady_now);
    const bool manual_override = armed_ && selected_source_ != "manual" && manual_active &&
      !hybrid_altitude_hold;
    active_source_ = hybrid_altitude_hold ? "pid+manual" :
      (manual_override ? "manual" : selected_source_);
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
      const auto altitude_manual = hybrid_altitude_hold ?
        altitude_manual_surge_yaw(*source_commands_.at("manual").message) :
        std::array<double, 8>{};
      const auto & selected_values = source_commands_.at(
        hybrid_altitude_hold ? "pid" : active_source_).message->action;
      for (std::size_t i = 0; i < output_.size(); ++i) {
        const double requested = hybrid_altitude_hold && i >= 4U ?
          altitude_manual[i] : static_cast<double>(selected_values[i]);
        output_[i] = requested * action_limit_;
      }
      message_ = manual_override ?
        "armed " + selected_source_ + "; manual LB override" :
        (hybrid_altitude_hold ? "armed pid; altitude hold + manual control" :
        "armed " + selected_source_);
    } else {output_.fill(0.0);}

    output_enabled_ = output_allowed && idle_reason.empty();
  }

  ThrusterCommand make_command(bool enable)
  {
    ThrusterCommand command;
    command.header.stamp = now(); command.header.frame_id = "base_link";
    for (std::size_t i = 0; i < output_.size(); ++i) {
      command.action[i] = static_cast<float>(output_[i]);
    }
    command.enable = enable; command.armed = armed_;
    command.arm_generation = arm_generation_;
    command.source = "command_authority:" + active_source_;
    return command;
  }

  void publish_command(bool enable)
  {
    command_pub_->publish(make_command(enable));
  }

  void publish_status(SteadyClock::time_point steady_now)
  {
    AuthorityStatus status;
    status.header.stamp = now(); status.selected_source = selected_source_;
    status.armed = armed_; status.arm_generation = arm_generation_;
    status.abort_active = abort_active_; status.fault_latched = fault_latched_;
    status.fault_code = fault_code_; status.message = message_;
    const auto selected = source_commands_.find(selected_source_);
    status.selected_command_age_s = age_s(
      selected == source_commands_.end() ? std::optional<SteadyClock::time_point>{} :
      std::optional<SteadyClock::time_point>{selected->second.stamp}, steady_now);
    status.body_state_age_s = age_s(
      body_ ? std::optional<SteadyClock::time_point>{body_->stamp} : std::nullopt, steady_now);
    status.target_age_s = age_s(
      target_ ? std::optional<SteadyClock::time_point>{target_->stamp} : std::nullopt, steady_now);
    status.action_limit = action_limit_; status.action_slew_rate = 0.0;
    status.localization_source = body_ ? body_->message->localization_source : "";
    status_pub_->publish(status);
  }

  const std::unordered_map<std::string, std::string> expected_producers_{
    {"manual", "web_operator"}, {"pid", "pid_controller"}, {"rl", "t60_policy"}};
  std::unordered_map<std::string, TimedSourceCommand> source_commands_;
  std::optional<TimedBody> body_;
  std::optional<TimedTarget> target_;
  std::optional<SteadyClock::time_point> safety_stamp_;
  std::array<double, 8> output_{};
  std::string selected_source_{"manual"}, active_source_{"manual"};
  std::string fault_code_, message_{"disarmed"};
  bool armed_{false}, abort_active_{false}, fault_latched_{false};
  bool output_enabled_{false};
  bool absolute_localization_seen_{false}, allow_rl_{false};
  std::uint64_t arm_generation_{0U};
  double evaluation_rate_hz_{}, publish_rate_hz_{}, status_rate_hz_{};
  double source_command_timeout_s_{}, state_timeout_s_{}, target_timeout_s_{}, safety_timeout_s_{};
  double action_limit_{};

  rclcpp::Publisher<ThrusterCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<AuthorityStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<ThrusterCommand>::SharedPtr manual_command_sub_;
  rclcpp::Subscription<ThrusterCommand>::SharedPtr pid_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rl_action_sub_;
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
