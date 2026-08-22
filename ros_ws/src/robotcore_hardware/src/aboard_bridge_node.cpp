#include "robotcore_hardware/aboard_protocol.hpp"

#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robotcore_interfaces/msg/april_tag_pose_estimate.hpp>
#include <robotcore_interfaces/msg/board_status.hpp>
#include <robotcore_interfaces/msg/board_runtime.hpp>
#include <robotcore_interfaces/msg/body_state.hpp>
#include <robotcore_interfaces/msg/thruster_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robotcore_hardware
{
namespace
{
constexpr std::uint8_t kKnownStatusFlags =
  kStatusFlagSessionEstablished | kStatusFlagOutputsEnabled |
  kStatusFlagFailsafe | kStatusFlagImuCalibrating |
  kStatusFlagImuCalibrationOk | kStatusFlagImuCalibrationFail;
constexpr std::uint16_t kMinimumReportedPwmUs = 1000U;
constexpr std::uint16_t kMaximumReportedPwmUs = 2000U;
constexpr std::uint32_t kMaximumAckSequenceLag = 8U;
constexpr std::size_t kDispatchHistoryDepth = 16U;
constexpr auto kCommandPeriod = 20ms;
constexpr auto kReconnectDelay = 1s;
constexpr auto kWriteDeadline = 50ms;
constexpr auto kShutdownDeadline = 150ms;
constexpr auto kImuCalibrationStartTimeout = 3s;
constexpr auto kImuCalibrationOverallTimeout = 25s;
constexpr char kAuthoritySourcePrefix[] = "command_authority:";
constexpr double kDegreesToRadians = 0.017453292519943295769;
constexpr double kTwoPi = 6.283185307179586476925;
constexpr std::int64_t kHeadingAlignmentMaximumSkewNs = 100000000LL;

struct RawHeadingSample
{
  std::int64_t stamp_ns{};
  double yaw_rad{};
};

std::array<double, 4> quaternion_wxyz_from_rpy(const std::array<double, 3> & rpy)
{
  const double cr = std::cos(0.5 * rpy[0]);
  const double sr = std::sin(0.5 * rpy[0]);
  const double cp = std::cos(0.5 * rpy[1]);
  const double sp = std::sin(0.5 * rpy[1]);
  const double cy = std::cos(0.5 * rpy[2]);
  const double sy = std::sin(0.5 * rpy[2]);
  return {
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy};
}

bool sequence_newer(std::uint32_t candidate, std::uint32_t reference)
{
  const auto delta = static_cast<std::uint32_t>(candidate - reference);
  return delta != 0U && delta < 0x80000000U;
}

bool generation_newer(std::uint64_t candidate, std::uint64_t reference)
{
  const auto delta = static_cast<std::uint64_t>(candidate - reference);
  return delta != 0U && delta < 0x8000000000000000ULL;
}

bool starts_with(const std::string & text, const char * prefix)
{
  const std::string expected(prefix);
  return text.size() >= expected.size() && text.compare(0U, expected.size(), expected) == 0;
}

bool board_status_semantically_valid(const BoardStatusFrame & status, std::string & reason)
{
  if ((status.flags & static_cast<std::uint8_t>(~kKnownStatusFlags)) != 0U) {
    reason = "unknown board-status flag";
    return false;
  }
  if (status.safety_reason > kMaximumSafetyReason) {
    reason = "unknown board safety reason";
    return false;
  }
  if (!board_status_reason_flags_consistent(status)) {
    reason = "board safety reason contradicts session/failsafe flags";
    return false;
  }
  const auto calibration_flags = static_cast<std::uint8_t>(status.flags &
    (kStatusFlagImuCalibrating | kStatusFlagImuCalibrationOk |
    kStatusFlagImuCalibrationFail));
  if (calibration_flags != 0U && (calibration_flags & (calibration_flags - 1U)) != 0U) {
    reason = "contradictory IMU calibration flags";
    return false;
  }
  if ((status.flags & kStatusFlagImuCalibrating) != 0U &&
    (status.flags & kStatusFlagOutputsEnabled) != 0U)
  {
    reason = "outputs enabled during IMU calibration";
    return false;
  }
  if (status.boot_id == 0U) {
    reason = "zero board boot challenge";
    return false;
  }
  for (const auto pwm : status.pwm_us) {
    if (pwm < kMinimumReportedPwmUs || pwm > kMaximumReportedPwmUs) {
      reason = "board PWM command echo outside firmware limits";
      return false;
    }
  }
  return true;
}
}  // namespace

class AboardBridgeNode final : public rclcpp::Node
{
public:
  AboardBridgeNode()
  : Node("aboard_bridge_node"),
    serial_(io_),
    reconnect_timer_io_(io_),
    write_deadline_io_(io_),
    shutdown_deadline_io_(io_),
    work_(boost::asio::make_work_guard(io_)),
    updater_(this)
  {
    port_ = declare_parameter<std::string>(
      "serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A033320-if00");
    const auto requested_baud = declare_parameter<std::int64_t>("baud", 115200);
    baud_ = static_cast<int>(std::clamp<std::int64_t>(requested_baud, 1, 4000000));
    const auto requested_span_us = declare_parameter<std::int64_t>("span_us", 500);
    span_us_ = static_cast<int>(std::clamp<std::int64_t>(requested_span_us, 1, 500));
    if (span_us_ != requested_span_us) {
      RCLCPP_WARN(
        get_logger(), "span_us %ld clamped to the firmware-safe range [1, 500]",
        requested_span_us);
    }
    command_timeout_ms_ = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("command_timeout_ms", 150)));
    heartbeat_timeout_ms_ = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("heartbeat_timeout_ms", 250)));
    authority_node_name_ = declare_parameter<std::string>(
      "authority_node_name", "command_authority");
    authority_node_namespace_ = declare_parameter<std::string>(
      "authority_node_namespace", "/");
    const double imu_yaw_offset_deg = declare_parameter<double>("imu_yaw_offset_deg", 0.0);
    if (!std::isfinite(imu_yaw_offset_deg)) {
      throw std::invalid_argument("imu_yaw_offset_deg must be finite");
    }
    imu_yaw_offset_rad_.store(
      std::remainder(imu_yaw_offset_deg * kDegreesToRadians, kTwoPi),
      std::memory_order_relaxed);
    gyro_stddev_ = declare_parameter<double>("imu_angular_velocity_stddev_rps", 0.05);
    accel_stddev_ = declare_parameter<double>("imu_linear_acceleration_stddev_mps2", 0.5);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/sensors/external_imu", rclcpp::SensorDataQoS().keep_last(8));
    status_pub_ = create_publisher<robotcore_interfaces::msg::BoardStatus>("/hardware/board_status", 10);
    runtime_pub_ = create_publisher<robotcore_interfaces::msg::BoardRuntime>(
      "/hardware/board_runtime", 10);
    command_sub_ = create_subscription<robotcore_interfaces::msg::ThrusterCommand>(
      "/control/thruster_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const robotcore_interfaces::msg::ThrusterCommand::SharedPtr message)
      {
        on_command(message);
      });
    tag_pose_sub_ = create_subscription<robotcore_interfaces::msg::AprilTagPoseEstimate>(
      "/localization/apriltag_pose", rclcpp::SensorDataQoS().keep_last(4),
      std::bind(&AboardBridgeNode::on_tag_pose, this, std::placeholders::_1));
    body_state_sub_ = create_subscription<robotcore_interfaces::msg::BodyState>(
      "/robot/body_state", rclcpp::QoS(10),
      std::bind(&AboardBridgeNode::on_body_state, this, std::placeholders::_1));
    imu_calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "/hardware/aboard/calibrate_gyro",
      std::bind(
        &AboardBridgeNode::calibrate_imu_gyro, this,
        std::placeholders::_1, std::placeholders::_2));
    command_timer_ = create_wall_timer(kCommandPeriod, std::bind(&AboardBridgeNode::write_command, this));
      status_timer_ = create_wall_timer(100ms, std::bind(&AboardBridgeNode::publish_status, this));
      updater_.setHardwareID("aboard-uart6");
      updater_.add("Aquaboard serial and IMU", this, &AboardBridgeNode::diagnose);
      diagnostic_timer_ = create_wall_timer(1s, [this]() {updater_.force_update();});

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      session_id_ = make_session_id();
      transition_reason_ = "process start";
    }
    imu_publish_thread_ = std::thread([this]() {imu_publish_loop();});
    io_thread_ = std::thread([this]() {io_.run();});
    boost::asio::post(io_, [this]() {open_serial_io();});
    RCLCPP_INFO(
      get_logger(),
      "Aquaboard protocol v2 bridge started; PWM channels 8-15 are command echoes, not motor feedback");
  }

  ~AboardBridgeNode() override
  {
    shutting_down_.store(true);
      if (command_timer_) {command_timer_->cancel();}
      if (status_timer_) {status_timer_->cancel();}
      if (diagnostic_timer_) {diagnostic_timer_->cancel();}

    shutdown_promise_ = std::make_shared<std::promise<void>>();
    auto complete = shutdown_promise_->get_future();
    boost::asio::post(io_, [this]() {begin_shutdown_io();});
    if (complete.wait_for(500ms) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for the UART I/O thread to close");
    }
    work_.reset();
    io_.stop();
    if (io_thread_.joinable()) {io_thread_.join();}
    {
      std::lock_guard<std::mutex> lock(imu_queue_wait_mutex_);
      imu_publish_stop_.store(true);
    }
    imu_queue_cv_.notify_one();
    if (imu_publish_thread_.joinable()) {imu_publish_thread_.join();}
  }

private:
  struct QueuedImu
  {
    ImuFrame sample;
    std::int64_t stamp_ns{};
    std::int64_t arrival_ns{};
    std::int64_t steady_arrival_ns{};
    std::uint64_t link_generation{};
  };

  struct WriteRequest
  {
    std::array<std::uint8_t, kCommandV2FrameSize> frame{};
    std::array<std::int16_t, kThrusterChannels> offsets{};
    std::uint32_t boot_id{};
    std::uint32_t session_id{};
    std::uint32_t sequence{};
    bool enable{false};
    bool calibrate_imu_gyro{false};
    bool shutdown{false};
  };

  struct StateSnapshot
  {
    BoardStatusFrame board{};
    RuntimeFrame runtime{};
    bool have_status{false};
    bool have_runtime{false};
    bool semantic_fault{false};
    bool session_acknowledged{false};
    bool ack_fault{false};
    bool command_valid{false};
    bool command_armed{false};
    bool arm_authorized{false};
    bool saw_disarmed{false};
    bool publisher_gate{false};
    bool authority_endpoint{false};
    bool command_timeout_latched{false};
    bool imu_calibration_requested{false};
    bool imu_calibration_seen_active{false};
    bool imu_calibration_local_failed{false};
    bool have_dispatched{false};
    bool have_ack{false};
    bool have_boot_challenge{false};
    std::size_t publisher_count{};
    std::uint64_t arm_generation{};
    std::uint64_t authorized_generation{};
    std::uint64_t authority_epoch{};
    std::uint32_t boot_challenge{};
    std::uint32_t session_id{};
    std::uint32_t last_dispatched{};
    std::uint32_t last_ack_received{};
    std::uint32_t last_ack_applied{};
    std::int64_t last_status_ns{};
    std::int64_t last_runtime_ns{};
    std::uint64_t runtime_generation{};
    std::int64_t last_command_ns{};
    std::int64_t imu_calibration_request_ns{};
    std::string command_source;
    std::string transition_reason;
  };

  struct DispatchedCommand
  {
    CommandFrame command;
  };

  static std::int64_t steady_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static std::uint32_t make_session_id()
  {
    static std::atomic<std::uint32_t> nonce{0U};
    std::random_device random;
    const auto steady = static_cast<std::uint64_t>(steady_now_ns());
    auto value = static_cast<std::uint32_t>(steady) ^
      static_cast<std::uint32_t>(steady >> 32U) ^
      (static_cast<std::uint32_t>(random()) << 1U) ^
      static_cast<std::uint32_t>(::getpid()) ^ ++nonce;
    if (value == 0U) {value = 1U;}
    return value;
  }

  void rotate_session_locked(const std::string & reason, bool ack_fault = false)
  {
    const auto previous = session_id_;
    do {
      session_id_ = make_session_id();
    } while (session_id_ == 0U || session_id_ == previous);
    command_sequence_ = 0U;
    have_dispatched_sequence_ = false;
    last_dispatched_sequence_ = 0U;
    have_handshake_sequence_ = false;
    handshake_sequence_ = 0U;
    have_ack_baseline_ = false;
    last_ack_received_ = 0U;
    last_ack_applied_ = 0U;
    session_acknowledged_ = false;
    ack_fault_latched_ = ack_fault;
    dispatched_history_.clear();

    // A transport epoch is never allowed to inherit authority.  A valid
    // disarmed message must be observed after this point, followed by a
    // strictly newer arm generation.
    arm_authorized_ = false;
    saw_disarmed_after_session_ = false;
    disarmed_generation_ = 0U;
    transition_reason_ = reason;
    ++session_rotations_;
  }

  template<typename OptionT>
  bool set_serial_option_io(const OptionT & option, const char * name)
  {
    boost::system::error_code error;
    serial_.set_option(option, error);
    if (!error) {return true;}
    RCLCPP_ERROR(get_logger(), "Cannot set Aquaboard %s: %s", name, error.message().c_str());
    return false;
  }

  bool configure_serial_io()
  {
    if (!set_serial_option_io(boost::asio::serial_port_base::baud_rate(baud_), "baud rate") ||
      !set_serial_option_io(
        boost::asio::serial_port_base::character_size(8), "character size") ||
      !set_serial_option_io(
        boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
        "parity") ||
      !set_serial_option_io(
        boost::asio::serial_port_base::stop_bits(
          boost::asio::serial_port_base::stop_bits::one),
        "stop bits") ||
      !set_serial_option_io(
        boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none),
        "flow control"))
    {
      return false;
    }

    termios options{};
    if (::tcgetattr(serial_.native_handle(), &options) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed for Aquaboard: %s", std::strerror(errno));
      return false;
    }
    ::cfmakeraw(&options);
    const auto forbidden_input = static_cast<tcflag_t>(
      IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    const auto forbidden_local = static_cast<tcflag_t>(
      ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_iflag &= static_cast<tcflag_t>(~forbidden_input);
    options.c_lflag &= static_cast<tcflag_t>(~forbidden_local);
    options.c_oflag &= static_cast<tcflag_t>(~OPOST);
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
    options.c_cflag |= CS8;
    options.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
#ifdef CRTSCTS
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    if (::tcsetattr(serial_.native_handle(), TCSANOW, &options) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed for Aquaboard: %s", std::strerror(errno));
      return false;
    }

    termios verified{};
    if (::tcgetattr(serial_.native_handle(), &verified) != 0) {
      RCLCPP_ERROR(get_logger(), "Cannot verify Aquaboard termios: %s", std::strerror(errno));
      return false;
    }
    const bool raw_input = (verified.c_iflag & forbidden_input) == 0U;
    const bool raw_local = (verified.c_lflag & forbidden_local) == 0U;
    const bool raw_output = (verified.c_oflag & OPOST) == 0U;
    bool eight_n_one = (verified.c_cflag & CSIZE) == CS8 &&
      (verified.c_cflag & static_cast<tcflag_t>(PARENB | CSTOPB)) == 0U;
    bool no_software_flow =
      (verified.c_iflag & static_cast<tcflag_t>(IXON | IXOFF | IXANY)) == 0U;
    bool no_hardware_flow = true;
#ifdef CRTSCTS
    no_hardware_flow = (verified.c_cflag & CRTSCTS) == 0U;
#endif
    const bool receiver_enabled =
      (verified.c_cflag & static_cast<tcflag_t>(CLOCAL | CREAD)) ==
      static_cast<tcflag_t>(CLOCAL | CREAD);
    boost::asio::serial_port_base::baud_rate verified_baud;
    boost::system::error_code baud_error;
    serial_.get_option(verified_baud, baud_error);
    const bool baud_matches = !baud_error &&
      verified_baud.value() == static_cast<unsigned int>(baud_);
    if (!raw_input || !raw_local || !raw_output || !eight_n_one || !no_software_flow ||
      !no_hardware_flow || !receiver_enabled || !baud_matches)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Aquaboard termios verification failed (raw_input=%d raw_local=%d raw_output=%d "
        "8N1=%d sw_flow_off=%d hw_flow_off=%d receiver=%d baud=%d)",
        raw_input, raw_local, raw_output, eight_n_one, no_software_flow, no_hardware_flow,
        receiver_enabled, baud_matches);
      return false;
    }
    if (::tcflush(serial_.native_handle(), TCIFLUSH) != 0) {
      RCLCPP_ERROR(get_logger(), "Cannot flush stale Aquaboard input: %s", std::strerror(errno));
      return false;
    }
    return true;
  }

  void open_serial_io()
  {
    if (shutting_down_.load() || link_open_) {return;}
    boost::system::error_code error;
    serial_.open(port_, error);
    if (error) {
      connected_.store(false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot open Aquaboard %s: %s",
        port_.c_str(), error.message().c_str());
      schedule_reconnect_io();
      return;
    }
    if (::flock(serial_.native_handle(), LOCK_EX | LOCK_NB) != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Aquaboard serial device is already owned: %s",
        port_.c_str());
      close_serial_io();
      schedule_reconnect_io();
      return;
    }
    if (!configure_serial_io()) {
      close_serial_io();
      schedule_reconnect_io();
      return;
    }

    rx_buffer_.clear();
    clock_mapper_.reset();
    have_counter_ = false;
    have_mcu_tick_ = false;
    ++link_generation_io_;
    link_generation_.store(link_generation_io_);
    std::uint32_t new_session = 0U;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_status_ns_ = 0;
      status_semantic_fault_ = false;
      heartbeat_timeout_latched_ = false;
      have_link_boot_challenge_ = false;
      link_boot_id_ = 0U;
      rotate_session_locked("serial link opened");
      new_session = session_id_;
    }
    link_open_ = true;
    connected_.store(true);
    start_read_io();
    dispatch_neutral_current_session_io();
    RCLCPP_INFO(
      get_logger(), "Opened Aquaboard UART6 %s at %d baud; new session %08x",
      port_.c_str(), baud_, new_session);
  }

  void schedule_reconnect_io()
  {
    if (shutting_down_.load()) {return;}
    reconnect_timer_io_.expires_after(kReconnectDelay);
    reconnect_timer_io_.async_wait([this](const boost::system::error_code & error) {
      if (!error && !shutting_down_.load()) {open_serial_io();}
    });
  }

  void close_serial_io()
  {
    boost::system::error_code ignored;
    if (serial_.is_open()) {
      serial_.cancel(ignored);
      ignored.clear();
      serial_.close(ignored);
    }
  }

  void handle_io_fault_io(const char * operation, const boost::system::error_code & error)
  {
    if (!link_open_) {return;}
    link_open_ = false;
    connected_.store(false);
    ++link_generation_io_;
    link_generation_.store(link_generation_io_);
    pending_write_.reset();
    boost::system::error_code ignored;
    write_deadline_io_.cancel(ignored);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_status_ns_ = 0;
      status_semantic_fault_ = false;
      rotate_session_locked(std::string("serial ") + operation + " fault");
    }
    close_serial_io();
    if (!shutting_down_.load()) {
      if (error != boost::asio::error::operation_aborted) {
        RCLCPP_ERROR(
          get_logger(), "Aquaboard %s failed: %s", operation, error.message().c_str());
      }
      schedule_reconnect_io();
    } else {
      finish_shutdown_io();
    }
  }

  void start_read_io()
  {
    if (!link_open_ || shutdown_finished_io_) {return;}
    serial_.async_read_some(
      boost::asio::buffer(read_chunk_),
      [this](const boost::system::error_code & error, std::size_t size) {
        if (error) {
          handle_io_fault_io("read", error);
          return;
        }
        rx_buffer_.insert(rx_buffer_.end(), read_chunk_.begin(), read_chunk_.begin() + size);
        parse_rx_io();
        start_read_io();
      });
  }

  void parse_rx_io()
  {
    while (rx_buffer_.size() >= 3U) {
      const auto begin = std::find(rx_buffer_.begin(), rx_buffer_.end(), kFrameHead);
      if (begin != rx_buffer_.begin()) {rx_buffer_.erase(rx_buffer_.begin(), begin);}
      if (rx_buffer_.size() < 3U) {return;}

      std::size_t length = 0U;
      if (rx_buffer_[1] == kStatusV2Type) {
        length = kStatusV2FrameSize;
      } else if (rx_buffer_[1] == 0xF8U) {
        if (rx_buffer_[2] == kImuFrameNumber) {
          length = kImuV2FrameSize;
        } else if (rx_buffer_[2] == kRuntimeFrameNumber) {
          length = kRuntimeV1FrameSize;
        } else {
          ++imu_version_errors_;
          rx_buffer_.erase(rx_buffer_.begin());
          continue;
        }
      } else {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }
      if (rx_buffer_.size() < length) {return;}

      bool accepted = false;
      if (rx_buffer_[1] == kStatusV2Type) {
        const auto board_status = parse_board_status_v2(rx_buffer_.data(), length);
        if (board_status) {
          accept_board_status_io(*board_status);
          accepted = true;
        } else {
          ++status_crc_errors_;
        }
      } else if (rx_buffer_[2] == kRuntimeFrameNumber) {
        const auto runtime = parse_runtime_v1(rx_buffer_.data(), length);
        if (runtime) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_runtime_ = *runtime;
          have_runtime_ = true;
          last_runtime_ns_ = steady_now_ns();
          ++runtime_generation_;
          accepted = true;
        } else {
          ++runtime_crc_errors_;
        }
      } else if (rx_buffer_[3] != 0x02U) {
        ++imu_version_errors_;
        accepted = true;
      } else if ((rx_buffer_[4] & 0x01U) == 0U) {
        ++imu_invalid_flags_;
        accepted = true;
      } else {
        const auto sample = parse_imu_v2(rx_buffer_.data(), length);
        if (sample) {accepted = handle_imu_io(*sample);}
        else {++imu_crc_errors_;}
      }

      if (accepted) {rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + length);}
      else {rx_buffer_.erase(rx_buffer_.begin());}
    }
    if (rx_buffer_.size() > 4096U) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 64);
    }
  }

  void accept_board_status_io(const BoardStatusFrame & board)
  {
    std::string semantic_reason;
    if (!board_status_semantically_valid(board, semantic_reason)) {
      ++status_semantic_errors_;
      bool newly_faulted = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        newly_faulted = !status_semantic_fault_;
        if (newly_faulted) {
          rotate_session_locked("invalid board-status semantics", true);
        }
        status_semantic_fault_ = true;
      }
      if (newly_faulted) {
        RCLCPP_ERROR(
          get_logger(), "Rejected semantically invalid Aquaboard status: %s",
          semantic_reason.c_str());
        dispatch_neutral_current_session_io();
      }
      return;
    }

    const auto now = steady_now_ns();
    std::string fault_reason;
    bool rotated = false;
    bool shutdown_neutral_applied = false;
    bool calibration_completed = false;
    bool calibration_failed = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      status_semantic_fault_ = false;
      heartbeat_timeout_latched_ = false;

      if (!have_link_boot_challenge_) {
        const bool boot_changed = have_boot_identity_ && board.boot_id != boot_identity_;
        const bool tick_rolled_back = have_boot_identity_ && !boot_changed &&
          have_identity_tick_ && board.board_tick_ms < last_identity_tick_ &&
          static_cast<std::uint32_t>(last_identity_tick_ - board.board_tick_ms) <= 0x80000000U;
        if (boot_changed || tick_rolled_back) {
          ++board_reset_events_;
        }
        have_link_boot_challenge_ = true;
        link_boot_id_ = board.boot_id;
        have_boot_identity_ = true;
        boot_identity_ = board.boot_id;
        last_identity_tick_ = board.board_tick_ms;
        have_identity_tick_ = true;
        fault_reason = boot_changed ? "board boot_id changed while link was down" :
          (tick_rolled_back ? "board tick rolled backwards while link was down" :
          "fresh board boot challenge acquired");
        rotate_session_locked(fault_reason);
        rotated = true;
      } else if (board.boot_id != boot_identity_) {
        link_boot_id_ = board.boot_id;
        boot_identity_ = board.boot_id;
        last_identity_tick_ = board.board_tick_ms;
        have_identity_tick_ = true;
        ++board_reset_events_;
        fault_reason = "board boot_id changed";
        rotate_session_locked(fault_reason);
        rotated = true;
      } else {
        if (have_identity_tick_ && board.board_tick_ms < last_identity_tick_ &&
          static_cast<std::uint32_t>(last_identity_tick_ - board.board_tick_ms) <= 0x80000000U)
        {
          ++board_reset_events_;
          fault_reason = "board tick rolled backwards";
          rotate_session_locked(fault_reason);
          rotated = true;
        }
        last_identity_tick_ = board.board_tick_ms;
        have_identity_tick_ = true;
      }

      latest_status_ = board;
      have_status_ = true;
      last_status_ns_ = now;
      ++status_frames_;

      const bool calibration_active =
        (board.flags & kStatusFlagImuCalibrating) != 0U;
      if (imu_calibration_requested_ && calibration_active) {
        imu_calibration_seen_active_ = true;
      }
      if (imu_calibration_requested_ && imu_calibration_seen_active_ && !calibration_active) {
        if ((board.flags & kStatusFlagImuCalibrationOk) != 0U) {
          calibration_completed = true;
          imu_calibration_requested_ = false;
          transition_reason_ = "external IMU gyro calibration succeeded";
        } else if ((board.flags & kStatusFlagImuCalibrationFail) != 0U) {
          calibration_failed = true;
          imu_calibration_requested_ = false;
          transition_reason_ = "external IMU gyro calibration failed";
        }
      }

      const bool reported_session =
        (board.flags & kStatusFlagSessionEstablished) != 0U &&
        board.session_id == session_id_;
      if (!rotated && session_acknowledged_ && !reported_session) {
        fault_reason = "board dropped the acknowledged session";
        rotate_session_locked(fault_reason);
        rotated = true;
      }

      if (!rotated && reported_session) {
        bool ack_invalid = !have_dispatched_sequence_;
        if (!ack_invalid && sequence_newer(board.applied_sequence, board.received_sequence)) {
          ack_invalid = true;
          fault_reason = "board applied sequence is newer than received sequence";
        }
        if (!ack_invalid && sequence_newer(board.received_sequence, last_dispatched_sequence_)) {
          ack_invalid = true;
          fault_reason = "board ACK is newer than the last dispatched command";
        }
        if (!ack_invalid && have_ack_baseline_ &&
          (sequence_newer(last_ack_received_, board.received_sequence) ||
          sequence_newer(last_ack_applied_, board.applied_sequence)))
        {
          ack_invalid = true;
          fault_reason = "board ACK sequence moved backwards";
        }
        if (!ack_invalid) {
          const auto received_lag = static_cast<std::uint32_t>(
            last_dispatched_sequence_ - board.received_sequence);
          if (received_lag >= 0x80000000U ||
            received_lag > kMaximumAckSequenceLag)
          {
            ack_invalid = true;
            fault_reason = "board received-sequence ACK exceeded lag budget";
          }
        }
        if (!ack_invalid) {
          const auto applied_lag = static_cast<std::uint32_t>(
            last_dispatched_sequence_ - board.applied_sequence);
          if (applied_lag >= 0x80000000U ||
            applied_lag > kMaximumAckSequenceLag)
          {
            ack_invalid = true;
            fault_reason = "board applied-sequence ACK exceeded lag budget";
          }
        }
        const CommandFrame * received_command = nullptr;
        const CommandFrame * applied_command = nullptr;
        if (!ack_invalid) {
          for (const auto & dispatched : dispatched_history_) {
            if (dispatched.command.sequence == board.received_sequence) {
              received_command = &dispatched.command;
            }
            if (dispatched.command.sequence == board.applied_sequence) {
              applied_command = &dispatched.command;
            }
          }
          if (received_command == nullptr || applied_command == nullptr ||
            received_command->boot_id != board.boot_id ||
            received_command->session_id != board.session_id)
          {
            ack_invalid = true;
            fault_reason = "board ACK does not identify a dispatched boot-bound command";
          }
        }
        if (!ack_invalid && !board_status_matches_applied_command(board, *applied_command)) {
          ack_invalid = true;
          fault_reason = "board applied ACK/PWM echo does not match dispatched command";
        }

        if (ack_invalid) {
          ++ack_fault_events_;
          if (fault_reason.empty()) {fault_reason = "board ACK before any command dispatch";}
          rotate_session_locked(fault_reason, true);
          rotated = true;
        } else {
          have_ack_baseline_ = true;
          last_ack_received_ = board.received_sequence;
          last_ack_applied_ = board.applied_sequence;
          const bool handshake_applied = have_handshake_sequence_ &&
            (board.applied_sequence == handshake_sequence_ ||
            sequence_newer(board.applied_sequence, handshake_sequence_));
          if (handshake_applied) {
            session_acknowledged_ = true;
            ack_fault_latched_ = false;
          }
        }
      }
      shutdown_neutral_applied = shutting_down_.load() && shutdown_neutral_queued_io_ &&
        !rotated && board.session_id == shutdown_session_id_io_ &&
        (board.flags & kStatusFlagSessionEstablished) != 0U &&
        (board.flags & kStatusFlagOutputsEnabled) == 0U &&
        (board.applied_sequence == shutdown_sequence_io_ ||
        sequence_newer(board.applied_sequence, shutdown_sequence_io_));
    }

    if (rotated) {
      RCLCPP_WARN(
        get_logger(), "Rotated Aquaboard control session: %s", fault_reason.c_str());
      dispatch_neutral_current_session_io();
    }
    if (shutdown_neutral_applied) {finish_shutdown_io();}
    if (calibration_completed) {
      RCLCPP_INFO(get_logger(), "External IMU saved gyro calibration succeeded");
    } else if (calibration_failed) {
      RCLCPP_ERROR(get_logger(), "External IMU saved gyro calibration failed");
    }
  }

  bool handle_imu_io(const ImuFrame & sample)
  {
    if (have_mcu_tick_ && sample.sample_tick_ms < last_mcu_tick_ &&
      static_cast<std::uint32_t>(last_mcu_tick_ - sample.sample_tick_ms) <= 0x80000000U)
    {
      clock_mapper_.reset();
      have_counter_ = false;
    }
    last_mcu_tick_ = sample.sample_tick_ms;
    have_mcu_tick_ = true;
    if (have_counter_) {
      const auto delta = static_cast<std::uint32_t>(sample.sample_counter - last_counter_);
      if (delta == 0U || delta > 0x80000000U) {
        ++imu_duplicate_or_backwards_;
        return true;
      }
      if (delta > 1U) {imu_sequence_gaps_ += delta - 1U;}
    }
    have_counter_ = true;
    last_counter_ = sample.sample_counter;
    const auto arrival = get_clock()->now().nanoseconds();
    const auto steady_arrival = steady_now_ns();
    const auto stamp = clock_mapper_.map(
      sample.sample_tick_ms, arrival, steady_arrival);
    if (clock_mapper_.take_ros_clock_discontinuity()) {
      ++ros_clock_discontinuities_;
      RCLCPP_WARN(
        get_logger(),
        "ROS/system clock discontinuity detected; IMU timestamps re-anchored");
    }
    if (!stamp) {
      ++imu_time_errors_;
      return true;
    }
    QueuedImu queued{
      sample, *stamp, arrival, steady_arrival, link_generation_io_};
    {
      // Pair the queue predicate update with the condition-variable mutex so
      // a notify cannot be lost between the consumer's predicate check and
      // its atomic transition into wait().
      std::lock_guard<std::mutex> lock(imu_queue_wait_mutex_);
      if (!imu_queue_.push(queued)) {
        ++imu_queue_overflows_;
        return true;
      }
      queue_high_water_.store(std::max(queue_high_water_.load(), imu_queue_.read_available()));
    }
    imu_queue_cv_.notify_one();
    return true;
  }

  void imu_publish_loop()
  {
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(imu_queue_wait_mutex_);
        imu_queue_cv_.wait(lock, [this]() {
          return imu_publish_stop_.load() || imu_queue_.read_available() != 0U;
        });
      }
      QueuedImu queued;
      const auto generation = link_generation_.load();
      while (imu_queue_.pop(queued)) {
        if (queued.link_generation != generation) {
          ++imu_stale_generation_drops_;
          continue;
        }
        publish_imu(queued);
      }
      if (imu_publish_stop_.load() && imu_queue_.read_available() == 0U) {return;}
    }
  }

  void publish_imu(const QueuedImu & queued)
  {
    const auto & sample = queued.sample;
    sensor_msgs::msg::Imu message;
    message.header.stamp = rclcpp::Time(queued.stamp_ns, RCL_ROS_TIME);
    message.header.frame_id = "base_link";
    if (sample.attitude_valid) {
      {
        std::lock_guard<std::mutex> lock(heading_history_mutex_);
        raw_heading_history_.push_back({queued.stamp_ns, sample.attitude_rpy_rad[2]});
        while (raw_heading_history_.size() > 256U) {raw_heading_history_.pop_front();}
      }
      auto aligned_rpy = imu_attitude_rpy_to_base_link(sample.attitude_rpy_rad);
      // Rotate the corrected base_link attitude into the map heading reference.
      aligned_rpy[2] = std::remainder(
        aligned_rpy[2] + imu_yaw_offset_rad_.load(std::memory_order_relaxed), kTwoPi);
      const auto orientation = quaternion_wxyz_from_rpy(aligned_rpy);
      message.orientation.w = orientation[0];
      message.orientation.x = orientation[1];
      message.orientation.y = orientation[2];
      message.orientation.z = orientation[3];
      // The vendor protocol supplies an attitude estimate but no covariance.
      // Per sensor_msgs/Imu, an all-zero covariance means covariance unknown.
    } else {
      message.orientation_covariance[0] = -1.0;
    }
    imu_attitude_valid_.store(sample.attitude_valid);
    const auto angular_velocity = imu_vector_to_base_link(sample.gyro_rad_s);
    const auto linear_acceleration = imu_vector_to_base_link(sample.accel_m_s2);
    message.angular_velocity.x = angular_velocity[0];
    message.angular_velocity.y = angular_velocity[1];
    message.angular_velocity.z = angular_velocity[2];
    message.linear_acceleration.x = linear_acceleration[0];
    message.linear_acceleration.y = linear_acceleration[1];
    message.linear_acceleration.z = linear_acceleration[2];
    message.angular_velocity_covariance[0] = gyro_stddev_ * gyro_stddev_;
    message.angular_velocity_covariance[4] = gyro_stddev_ * gyro_stddev_;
    message.angular_velocity_covariance[8] = gyro_stddev_ * gyro_stddev_;
    message.linear_acceleration_covariance[0] = accel_stddev_ * accel_stddev_;
    message.linear_acceleration_covariance[4] = accel_stddev_ * accel_stddev_;
    message.linear_acceleration_covariance[8] = accel_stddev_ * accel_stddev_;
    imu_pub_->publish(message);
    const auto published_at = get_clock()->now().nanoseconds();
    last_imu_arrival_steady_ns_.store(queued.steady_arrival_ns);
    last_imu_stamp_ros_ns_.store(queued.stamp_ns);
    {
      std::lock_guard<std::mutex> lock(imu_statistics_mutex_);
      imu_publish_times_.push_back(published_at);
      imu_transport_ms_.push_back((published_at - queued.stamp_ns) * 1e-6);
      while (imu_publish_times_.size() > 500U) {imu_publish_times_.pop_front();}
      while (imu_transport_ms_.size() > 500U) {imu_transport_ms_.pop_front();}
    }
    ++imu_frames_;
  }

  void on_tag_pose(
    const robotcore_interfaces::msg::AprilTagPoseEstimate::SharedPtr message)
  {
    const bool map_changed = last_tag_map_generation_ != 0U &&
      message->map_generation > last_tag_map_generation_;
    if (message->map_generation >= last_tag_map_generation_) {
      last_tag_map_generation_ = message->map_generation;
    }
    if (!message->relocalization_requested && !map_changed) {return;}

    heading_alignment_after_ros_ns_ =
      static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(message->header.stamp.nanosec);
    heading_alignment_pending_ = true;
    RCLCPP_INFO(
      get_logger(), "AprilTag relocalization received; waiting for a fresh absolute map pose");
  }

  void on_body_state(const robotcore_interfaces::msg::BodyState::SharedPtr message)
  {
    if (!heading_alignment_pending_ || !message->state_valid ||
      message->header.frame_id != "map")
    {
      return;
    }
    const auto body_stamp_ns =
      static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(message->header.stamp.nanosec);
    if (body_stamp_ns <= heading_alignment_after_ros_ns_) {return;}

    const auto & orientation = message->pose.orientation;
    const double body_yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    if (!std::isfinite(body_yaw)) {return;}

    RawHeadingSample closest;
    std::int64_t closest_skew_ns = std::numeric_limits<std::int64_t>::max();
    {
      std::lock_guard<std::mutex> lock(heading_history_mutex_);
      for (const auto & candidate : raw_heading_history_) {
        const auto skew_ns = candidate.stamp_ns >= body_stamp_ns ?
          candidate.stamp_ns - body_stamp_ns : body_stamp_ns - candidate.stamp_ns;
        if (skew_ns < closest_skew_ns) {
          closest = candidate;
          closest_skew_ns = skew_ns;
        }
      }
    }
    if (closest_skew_ns > kHeadingAlignmentMaximumSkewNs) {return;}

    const double offset = std::remainder(body_yaw - closest.yaw_rad, kTwoPi);
    imu_yaw_offset_rad_.store(offset, std::memory_order_relaxed);
    heading_alignment_pending_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Aligned external IMU heading to live map pose: offset %.3f deg (sample skew %.1f ms)",
      offset / kDegreesToRadians, closest_skew_ns * 1e-6);
  }

  void reset_arm_barrier_locked()
  {
    arm_authorized_ = false;
    saw_disarmed_after_session_ = false;
    disarmed_generation_ = 0U;
  }

  bool latch_command_timeout_locked(std::int64_t now_ns)
  {
    const bool expired = last_command_ns_ > 0 &&
      (now_ns - last_command_ns_) / 1000000LL > command_timeout_ms_;
    if (!expired || command_timeout_latched_) {return expired;}

    command_timeout_latched_ = true;
    command_valid_ = false;
    command_enabled_ = false;
    reset_arm_barrier_locked();
    transition_reason_ = "command_authority freshness timeout";
    ++command_timeout_events_;
    return true;
  }

  void update_publisher_gate_locked(std::size_t publisher_count)
  {
    command_publisher_count_ = publisher_count;
    if (publisher_count != 1U) {
      authority_publisher_identified_ = false;
      authority_refresh_requested_.store(true);
    } else if (!authority_publisher_identified_) {
      authority_refresh_requested_.store(true);
    }
    const bool valid_gate = publisher_count == 1U && authority_publisher_identified_;
    if (!valid_gate) {
      if (command_publisher_gate_ || command_valid_) {reset_arm_barrier_locked();}
      command_valid_ = false;
      command_enabled_ = false;
    }
    command_publisher_gate_ = valid_gate;
  }

  void refresh_authority_endpoint()
  {
    bool discovered_authority = false;
    std::size_t graph_publisher_count = 0U;

    try {
      const auto endpoints = get_publishers_info_by_topic(command_sub_->get_topic_name());
      graph_publisher_count = endpoints.size();
      if (endpoints.size() == 1U) {
        const auto & endpoint = endpoints.front();
        if (endpoint.node_name() == authority_node_name_ &&
          endpoint.node_namespace() == authority_node_namespace_)
        {
          discovered_authority = true;
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot inspect thruster authority endpoint; failing closed: %s", error.what());
    }

    const auto publisher_count = command_sub_->get_publisher_count();
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool valid_endpoint = publisher_count == 1U && graph_publisher_count == 1U &&
      discovered_authority;
    const bool endpoint_appeared = valid_endpoint && !authority_publisher_identified_;
    const bool endpoint_lost = !valid_endpoint &&
      authority_publisher_identified_;

    if (endpoint_appeared || endpoint_lost) {
      command_valid_ = false;
      command_enabled_ = false;
      reset_arm_barrier_locked();
      transition_reason_ = endpoint_appeared ?
        "command_authority publisher epoch changed" :
        "command_authority endpoint lost";
      ++authority_epoch_;
    }
    if (valid_endpoint) {
      authority_publisher_identified_ = true;
    } else {
      authority_publisher_identified_ = false;
    }
    update_publisher_gate_locked(publisher_count);
  }

  void on_command(const robotcore_interfaces::msg::ThrusterCommand::SharedPtr message)
  {
    const auto publisher_count = command_sub_->get_publisher_count();
    const auto now_ns = steady_now_ns();
    bool values_valid = true;
    for (const auto value : message->normalized) {
      if (!std::isfinite(value) || value < -1.0F || value > 1.0F) {
        values_valid = false;
        break;
      }
    }
    const bool source_valid = starts_with(message->source, kAuthoritySourcePrefix);

    std::lock_guard<std::mutex> lock(state_mutex_);
    (void)latch_command_timeout_locked(now_ns);
    update_publisher_gate_locked(publisher_count);
    const bool endpoint_valid = command_publisher_gate_;
    const bool message_valid = endpoint_valid && values_valid && source_valid;
    if (!message_valid) {
      if (!endpoint_valid) {authority_refresh_requested_.store(true);}
      command_valid_ = false;
      command_enabled_ = false;
      reset_arm_barrier_locked();
      ++invalid_command_events_;
      return;
    }

    command_valid_ = true;
    command_enabled_ = message->enable;
    command_armed_ = message->armed;
    command_arm_generation_ = message->arm_generation;
    command_source_ = message->source;
    command_offsets_.fill(0);
    for (std::size_t i = 0; i < message->normalized.size(); ++i) {
      command_offsets_[i] = static_cast<std::int16_t>(
        std::lround(static_cast<double>(message->normalized[i]) * span_us_));
    }
    last_command_ns_ = now_ns;
    command_timeout_latched_ = false;

    if (!message->armed) {
      arm_authorized_ = false;
      saw_disarmed_after_session_ = true;
      disarmed_generation_ = message->arm_generation;
    } else if (arm_authorized_ && message->arm_generation == authorized_generation_) {
      // Preserve an already-authorized generation across momentary dead-man
      // release; message.enable is intentionally independent of armed.
    } else if (saw_disarmed_after_session_ &&
      generation_newer(message->arm_generation, disarmed_generation_))
    {
      arm_authorized_ = true;
      authorized_generation_ = message->arm_generation;
      saw_disarmed_after_session_ = false;
    } else {
      arm_authorized_ = false;
      saw_disarmed_after_session_ = false;
    }
  }

  void calibrate_imu_gyro(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto now = steady_now_ns();
    const auto imu_arrival = last_imu_arrival_steady_ns_.load();
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool status_fresh = have_status_ && last_status_ns_ > 0 &&
      (now - last_status_ns_) / 1000000LL <= heartbeat_timeout_ms_;
    const bool imu_fresh = imu_arrival > 0 && (now - imu_arrival) <= 100000000LL;
    const bool board_outputs_disabled = have_status_ &&
      (latest_status_.flags & kStatusFlagOutputsEnabled) == 0U &&
      std::all_of(
      latest_status_.pwm_us.begin(), latest_status_.pwm_us.end(),
      [](std::uint16_t pwm) {return pwm == 1500U;});

    if (!connected_.load() || !status_fresh || status_semantic_fault_ ||
      !session_acknowledged_)
    {
      response->success = false;
      response->message = "Aquaboard link/session is not healthy";
      return;
    }
    if (!command_valid_ || command_armed_ || command_enabled_ || !board_outputs_disabled) {
      response->success = false;
      response->message = "disarm first and wait for all PWM outputs to report 1500 us";
      return;
    }
    if (!imu_fresh) {
      response->success = false;
      response->message = "external IMU stream is not fresh";
      return;
    }
    if (imu_calibration_requested_ ||
      (latest_status_.flags & kStatusFlagImuCalibrating) != 0U)
    {
      response->success = false;
      response->message = "external IMU gyro calibration is already active";
      return;
    }

    imu_calibration_requested_ = true;
    imu_calibration_seen_active_ = false;
    imu_calibration_local_failed_ = false;
    imu_calibration_request_ns_ = now;
    reset_arm_barrier_locked();
    transition_reason_ = "external IMU gyro calibration requested";
    response->success = true;
    response->message =
      "accepted; keep the vehicle level and completely still until completion "
      "(about 12 seconds); monitor /hardware/board_status";
  }

  bool host_enable_allowed_locked(std::int64_t now_ns) const
  {
    const bool status_fresh = have_status_ && last_status_ns_ > 0 &&
      (now_ns - last_status_ns_) / 1000000LL <= heartbeat_timeout_ms_;
    const bool board_session = have_status_ &&
      (latest_status_.flags & kStatusFlagSessionEstablished) != 0U &&
      latest_status_.session_id == session_id_;
    const bool board_safe = have_status_ &&
      (latest_status_.flags & kStatusFlagFailsafe) == 0U;
    const bool command_fresh = last_command_ns_ > 0 &&
      (now_ns - last_command_ns_) / 1000000LL <= command_timeout_ms_;
    const bool calibration_inhibit = imu_calibration_requested_ || (have_status_ &&
      (latest_status_.flags & kStatusFlagImuCalibrating) != 0U);
    return connected_.load() && status_fresh && !status_semantic_fault_ && board_session &&
      board_safe && session_acknowledged_ && !ack_fault_latched_ &&
      !calibration_inhibit &&
      command_publisher_gate_ && command_valid_ && command_fresh && command_enabled_ &&
      command_armed_ && arm_authorized_ && command_arm_generation_ == authorized_generation_;
  }

  void write_command()
  {
    const auto publisher_count = command_sub_->get_publisher_count();
    const auto now = steady_now_ns();
    WriteRequest request;
    bool heartbeat_rotated = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      (void)latch_command_timeout_locked(now);
      update_publisher_gate_locked(publisher_count);
      const bool status_timed_out = connected_.load() && have_status_ && last_status_ns_ > 0 &&
        (now - last_status_ns_) / 1000000LL > heartbeat_timeout_ms_;
      if (status_timed_out && !heartbeat_timeout_latched_) {
        rotate_session_locked("board-status heartbeat timeout");
        heartbeat_timeout_latched_ = true;
        heartbeat_rotated = true;
      }

      if (imu_calibration_requested_) {
        const auto elapsed = std::chrono::nanoseconds(now - imu_calibration_request_ns_);
        const bool start_timeout = !imu_calibration_seen_active_ &&
          elapsed > kImuCalibrationStartTimeout;
        const bool overall_timeout = elapsed > kImuCalibrationOverallTimeout;
        if (start_timeout || overall_timeout) {
          imu_calibration_requested_ = false;
          imu_calibration_local_failed_ = true;
          transition_reason_ = start_timeout ?
            "Aquaboard firmware did not acknowledge IMU calibration" :
            "external IMU calibration timed out";
          RCLCPP_ERROR(
            get_logger(), "%s", transition_reason_.c_str());
        }
      }

      request.session_id = session_id_;
      request.boot_id = have_link_boot_challenge_ ? link_boot_id_ : 0U;
      request.sequence = ++command_sequence_;
      request.calibrate_imu_gyro = imu_calibration_requested_;
      request.enable = host_enable_allowed_locked(now);
      if (request.enable) {request.offsets = command_offsets_;}
    }
    if (heartbeat_rotated) {
      RCLCPP_ERROR(get_logger(), "Aquaboard heartbeat timed out; session rotated and re-arm required");
    }
    enqueue_latest_write(std::move(request));
  }

  void enqueue_latest_write(WriteRequest request)
  {
    if (shutting_down_.load()) {return;}
    bool post_needed = false;
    {
      std::lock_guard<std::mutex> lock(outbound_mutex_);
      latest_outbound_ = std::move(request);
      if (!outbound_notification_posted_) {
        outbound_notification_posted_ = true;
        post_needed = true;
      }
    }
    if (post_needed) {
      boost::asio::post(io_, [this]() {drain_latest_write_io();});
    }
  }

  void drain_latest_write_io()
  {
    std::optional<WriteRequest> request;
    {
      std::lock_guard<std::mutex> lock(outbound_mutex_);
      outbound_notification_posted_ = false;
      request = std::move(latest_outbound_);
      latest_outbound_.reset();
    }
    if (!request || (shutting_down_.load() && !request->shutdown)) {return;}
    queue_write_io(std::move(*request));
  }

  void queue_write_io(WriteRequest request)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (request.session_id != session_id_) {return;}
      const auto expected_boot_id = have_link_boot_challenge_ ? link_boot_id_ : 0U;
      if (request.boot_id != expected_boot_id) {return;}
    }
    if (!active_write_) {
      start_write_io(std::move(request));
      return;
    }
    if (pending_write_ && pending_write_->session_id == request.session_id &&
      !sequence_newer(request.sequence, pending_write_->sequence))
    {
      return;
    }
    pending_write_ = std::move(request);
  }

  void start_write_io(WriteRequest request)
  {
    if (!link_open_ || !serial_.is_open()) {
      if (request.shutdown) {finish_shutdown_io();}
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (request.session_id != session_id_) {return;}
      if (have_dispatched_sequence_ &&
        !sequence_newer(request.sequence, last_dispatched_sequence_))
      {
        return;
      }
      if (request.enable && !host_enable_allowed_locked(steady_now_ns())) {
        request.enable = false;
        request.offsets.fill(0);
      }
      if (request.calibrate_imu_gyro) {
        request.enable = false;
        request.offsets.fill(0);
      }
      last_dispatched_sequence_ = request.sequence;
      have_dispatched_sequence_ = true;
      if (!request.enable && !have_handshake_sequence_) {
        handshake_sequence_ = request.sequence;
        have_handshake_sequence_ = true;
      }
      CommandFrame dispatched;
      dispatched.flags = request.enable ? kCommandFlagEnable : 0U;
      if (request.calibrate_imu_gyro) {
        dispatched.flags |= kCommandFlagImuGyroCalibrate;
      }
      dispatched.boot_id = request.boot_id;
      dispatched.session_id = request.session_id;
      dispatched.sequence = request.sequence;
      dispatched.offsets_us = request.offsets;
      dispatched_history_.push_back(DispatchedCommand{dispatched});
      while (dispatched_history_.size() > kDispatchHistoryDepth) {
        dispatched_history_.pop_front();
      }
    }

    request.frame = build_command_v2(
      request.boot_id, request.session_id, request.sequence, request.enable, request.offsets,
      request.calibrate_imu_gyro);
    auto active = std::make_shared<WriteRequest>(std::move(request));
    active_write_ = active;
    const auto token = ++write_token_;
    write_deadline_io_.expires_after(kWriteDeadline);
    write_deadline_io_.async_wait(
      [this, token](const boost::system::error_code & error) {
        if (!error && active_write_ && token == write_token_) {
          handle_io_fault_io("write deadline", boost::asio::error::timed_out);
        }
      });
    boost::asio::async_write(
      serial_, boost::asio::buffer(active->frame),
      [this, active, token](const boost::system::error_code & error, std::size_t) {
        if (active_write_ != active || token != write_token_) {return;}
        boost::system::error_code ignored;
        write_deadline_io_.cancel(ignored);
        active_write_.reset();
        if (error) {
          handle_io_fault_io("write", error);
          return;
        }
        if (active->shutdown) {
          // Keep the read side alive until status proves that the neutral
          // command crossed the MCU's PWM update boundary.  The bounded
          // shutdown timer remains the fallback for a broken return path.
          return;
        }
        if (pending_write_) {
          auto next = std::move(*pending_write_);
          pending_write_.reset();
          start_write_io(std::move(next));
        }
      });
  }

  void dispatch_neutral_current_session_io()
  {
    if (!link_open_ || shutting_down_.load()) {return;}
    WriteRequest neutral;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      neutral.session_id = session_id_;
      neutral.boot_id = have_link_boot_challenge_ ? link_boot_id_ : 0U;
      neutral.sequence = ++command_sequence_;
    }
    neutral.offsets.fill(0);
    neutral.enable = false;
    queue_write_io(std::move(neutral));
  }

  void begin_shutdown_io()
  {
    boost::system::error_code ignored;
    reconnect_timer_io_.cancel(ignored);
    {
      std::lock_guard<std::mutex> lock(outbound_mutex_);
      latest_outbound_.reset();
      outbound_notification_posted_ = false;
    }
    if (!link_open_ || !serial_.is_open()) {
      finish_shutdown_io();
      return;
    }

    WriteRequest neutral;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      neutral.session_id = session_id_;
      neutral.boot_id = have_link_boot_challenge_ ? link_boot_id_ : 0U;
      neutral.sequence = ++command_sequence_;
    }
    neutral.shutdown = true;
    neutral.enable = false;
    neutral.offsets.fill(0);
    shutdown_neutral_queued_io_ = true;
    shutdown_session_id_io_ = neutral.session_id;
    shutdown_sequence_io_ = neutral.sequence;
    shutdown_deadline_io_.expires_after(kShutdownDeadline);
    shutdown_deadline_io_.async_wait([this](const boost::system::error_code & error) {
      if (!error && !shutdown_finished_io_) {finish_shutdown_io();}
    });
    if (active_write_) {
      pending_write_ = std::move(neutral);
    } else {
      start_write_io(std::move(neutral));
    }
  }

  void finish_shutdown_io()
  {
    if (shutdown_finished_io_) {return;}
    shutdown_finished_io_ = true;
    link_open_ = false;
    connected_.store(false);
    boost::system::error_code ignored;
    reconnect_timer_io_.cancel(ignored);
    ignored.clear();
    write_deadline_io_.cancel(ignored);
    ignored.clear();
    shutdown_deadline_io_.cancel(ignored);
    close_serial_io();
    if (shutdown_promise_) {
      try {
        shutdown_promise_->set_value();
      } catch (const std::future_error &) {
      }
    }
  }

  StateSnapshot snapshot_state() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    StateSnapshot snapshot;
    snapshot.board = latest_status_;
    snapshot.runtime = latest_runtime_;
    snapshot.have_status = have_status_;
    snapshot.have_runtime = have_runtime_;
    snapshot.semantic_fault = status_semantic_fault_;
    snapshot.session_acknowledged = session_acknowledged_;
    snapshot.ack_fault = ack_fault_latched_;
    snapshot.command_valid = command_valid_;
    snapshot.command_armed = command_armed_;
    snapshot.arm_authorized = arm_authorized_;
    snapshot.saw_disarmed = saw_disarmed_after_session_;
    snapshot.publisher_gate = command_publisher_gate_;
    snapshot.authority_endpoint = authority_publisher_identified_;
    snapshot.command_timeout_latched = command_timeout_latched_;
    snapshot.imu_calibration_requested = imu_calibration_requested_;
    snapshot.imu_calibration_seen_active = imu_calibration_seen_active_;
    snapshot.imu_calibration_local_failed = imu_calibration_local_failed_;
    snapshot.imu_calibration_request_ns = imu_calibration_request_ns_;
    snapshot.have_dispatched = have_dispatched_sequence_;
    snapshot.have_ack = have_ack_baseline_;
    snapshot.have_boot_challenge = have_link_boot_challenge_;
    snapshot.publisher_count = command_publisher_count_;
    snapshot.arm_generation = command_arm_generation_;
    snapshot.authorized_generation = authorized_generation_;
    snapshot.authority_epoch = authority_epoch_;
    snapshot.boot_challenge = link_boot_id_;
    snapshot.session_id = session_id_;
    snapshot.last_dispatched = last_dispatched_sequence_;
    snapshot.last_ack_received = last_ack_received_;
    snapshot.last_ack_applied = last_ack_applied_;
    snapshot.last_status_ns = last_status_ns_;
    snapshot.last_runtime_ns = last_runtime_ns_;
    snapshot.runtime_generation = runtime_generation_;
    snapshot.last_command_ns = last_command_ns_;
    snapshot.command_source = command_source_;
    snapshot.transition_reason = transition_reason_;
    return snapshot;
  }

  void publish_status()
  {
    if (authority_refresh_requested_.exchange(false)) {
      refresh_authority_endpoint();
    }
    const auto now = get_clock()->now();
    const auto steady = steady_now_ns();
    const auto snapshot = snapshot_state();
    const bool heartbeat = connected_.load() && snapshot.have_status &&
      !snapshot.semantic_fault && snapshot.last_status_ns > 0 &&
      (steady - snapshot.last_status_ns) / 1000000LL <= heartbeat_timeout_ms_;
    const bool reported_session = snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagSessionEstablished) != 0U &&
      snapshot.board.session_id == snapshot.session_id;
    const bool reported_failsafe = snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagFailsafe) != 0U;
    const bool session_ok = heartbeat && reported_session &&
      snapshot.session_acknowledged && !snapshot.ack_fault;
    const bool command_fresh = snapshot.last_command_ns > 0 &&
      (steady - snapshot.last_command_ns) / 1000000LL <= command_timeout_ms_;
    const bool publisher_fault =
      !snapshot.publisher_gate || !snapshot.command_valid || !command_fresh;
    const bool rearm_required =
      snapshot.command_valid && snapshot.command_armed && !snapshot.arm_authorized;

    robotcore_interfaces::msg::BoardStatus status;
    status.header.stamp = now;
    status.connected = connected_.load();
    status.heartbeat_ok = heartbeat;
    status.session_established = session_ok;
    status.outputs_enabled = snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagOutputsEnabled) != 0U;
    status.imu_gyro_calibration_active = snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagImuCalibrating) != 0U;
    status.imu_gyro_calibration_succeeded = snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagImuCalibrationOk) != 0U;
    status.imu_gyro_calibration_failed = snapshot.imu_calibration_local_failed ||
      (snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagImuCalibrationFail) != 0U);
    status.failsafe_active = !heartbeat || !session_ok || snapshot.semantic_fault ||
      snapshot.ack_fault || reported_failsafe || publisher_fault ||
      rearm_required;
    status.protocol_version = snapshot.have_status ? snapshot.board.protocol_version : 0U;
    status.board_tick_ms = snapshot.board.board_tick_ms;
    status.boot_id = snapshot.board.boot_id;
    status.reset_cause = snapshot.board.reset_cause;
    status.control_session_id = snapshot.board.session_id;
    status.received_sequence = snapshot.board.received_sequence;
    status.applied_sequence = snapshot.board.applied_sequence;
    status.command_age_ms = snapshot.board.command_age_ms;
    status.rx_crc_errors = snapshot.board.rx_crc_errors;
    status.safety_reason = snapshot.board.safety_reason;
    if (snapshot.have_status) {
      status.pwm_us = snapshot.board.pwm_us;
    } else {
      status.pwm_us.fill(1500U);
    }
    status_pub_->publish(status);

    if (snapshot.have_runtime && snapshot.runtime_generation != published_runtime_generation_) {
      robotcore_interfaces::msg::BoardRuntime runtime;
      runtime.header.stamp = now;
      runtime.board_tick_ms = snapshot.runtime.board_tick_ms;
      runtime.cpu_idle_permille = snapshot.runtime.cpu_idle_permille;
      runtime.control_wcet_us = snapshot.runtime.control_wcet_us;
      runtime.uart_wcet_us = snapshot.runtime.uart_wcet_us;
      runtime.control_deadline_misses = snapshot.runtime.control_deadline_misses;
      runtime.uart_deadline_misses = snapshot.runtime.uart_deadline_misses;
      runtime.control_min_stack_words = snapshot.runtime.control_min_stack_words;
      runtime.uart_min_stack_words = snapshot.runtime.uart_min_stack_words;
      runtime.stack_overflow_count = snapshot.runtime.stack_overflow_count;
      runtime.watchdog_missed_windows = snapshot.runtime.watchdog_missed_windows;
      runtime.uart_rx_dma_errors = snapshot.runtime.uart_rx_dma_errors;
      runtime.uart_tx_drops = snapshot.runtime.uart_tx_drops;
      runtime_pub_->publish(runtime);
      published_runtime_generation_ = snapshot.runtime_generation;
    }

    }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const auto now = get_clock()->now().nanoseconds();
    const auto steady = steady_now_ns();
    const auto snapshot = snapshot_state();
    const auto last_arrival = last_imu_arrival_steady_ns_.load();
    const double imu_age_ms = last_arrival > 0 ?
      (steady - last_arrival) * 1e-6 : INFINITY;
    const auto stamp = last_imu_stamp_ros_ns_.load();
    const double imu_transport_ms = stamp > 0 ? (now - stamp) * 1e-6 : INFINITY;
    const double board_status_age_ms = snapshot.last_status_ns > 0 ?
      (steady - snapshot.last_status_ns) * 1e-6 : INFINITY;
    const bool connected = connected_.load();
    const bool imu_fresh = imu_age_ms <= 50.0;
    const bool imu_timestamp_valid =
      imu_transport_ms >= 0.0 && imu_transport_ms <= 50.0;
    const bool status_fresh = connected && snapshot.have_status && !snapshot.semantic_fault &&
      board_status_age_ms <= heartbeat_timeout_ms_;
    const double command_age_ms = snapshot.last_command_ns > 0 ?
      (steady - snapshot.last_command_ns) * 1e-6 : INFINITY;
    const bool command_fresh = command_age_ms <= command_timeout_ms_;
    const bool reported_session = snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagSessionEstablished) != 0U &&
      snapshot.board.session_id == snapshot.session_id;
    const bool session_ok = status_fresh && reported_session &&
      snapshot.session_acknowledged && !snapshot.ack_fault;
    const bool board_failsafe = snapshot.have_status &&
      (snapshot.board.flags & kStatusFlagFailsafe) != 0U;
    const bool rearm_required =
      snapshot.command_valid && snapshot.command_armed && !snapshot.arm_authorized;
    const bool healthy = connected && imu_fresh && imu_timestamp_valid && status_fresh && session_ok &&
      !board_failsafe && snapshot.publisher_gate && snapshot.command_valid && command_fresh &&
      !rearm_required;

    const std::string summary = !connected ? "serial disconnected" :
      (snapshot.semantic_fault ? "invalid board-status semantics" :
      (!status_fresh ? "no fresh protocol-v2 board status" :
      (!snapshot.publisher_gate ? "thruster command publisher count is not exactly one" :
      (!snapshot.command_valid ? "no valid command_authority command" :
      (!command_fresh ? "command_authority command timed out" :
      (snapshot.ack_fault ? "board command ACK fault" :
      (!session_ok ? "disabled handshake has not been applied by this session" :
      (rearm_required ? "fresh disarm and newer arm generation required" :
      (board_failsafe ? "board reports failsafe" :
      (!imu_fresh ? "no fresh frame-4 IMU" :
      (!imu_timestamp_valid ? "IMU timestamp is outside the current ROS clock epoch" :
      "command ACK and IMU healthy")))))))))));
    status.summary(
      healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      summary);

    status.add("imu_received_count", imu_frames_.load());
    status.add("imu_sequence_gaps", imu_sequence_gaps_.load());
    status.add("imu_crc_errors", imu_crc_errors_.load());
    status.add("imu_version_errors", imu_version_errors_.load());
    status.add("imu_invalid_flags", imu_invalid_flags_.load());
    status.add("imu_duplicate_or_backwards", imu_duplicate_or_backwards_.load());
    status.add("imu_time_errors", imu_time_errors_.load());
    status.add("ros_clock_discontinuities", ros_clock_discontinuities_.load());
    status.add("imu_age_ms", imu_age_ms);
    status.add("imu_timestamp_valid", imu_timestamp_valid);
    status.add("imu_transport_ms", imu_transport_ms);
    double imu_rate_hz = 0.0;
    double p95_ms = INFINITY;
    {
      std::lock_guard<std::mutex> lock(imu_statistics_mutex_);
      if (imu_publish_times_.size() >= 2U) {
        imu_rate_hz = static_cast<double>(imu_publish_times_.size() - 1U) * 1e9 /
          static_cast<double>(imu_publish_times_.back() - imu_publish_times_.front());
      }
      if (!imu_transport_ms_.empty()) {
        auto sorted = std::vector<double>(imu_transport_ms_.begin(), imu_transport_ms_.end());
        const auto index = static_cast<std::size_t>(std::ceil(0.95 * sorted.size())) - 1U;
        std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
        p95_ms = sorted[index];
      }
    }
    status.add("imu_rate_hz", imu_rate_hz);
    status.add("imu_transport_p95_ms", p95_ms);
    status.add("imu_queue_high_water", queue_high_water_.load());
    status.add("imu_queue_overflows", imu_queue_overflows_.load());
    status.add("imu_stale_link_generation_drops", imu_stale_generation_drops_.load());
    status.add("board_status_frames", status_frames_.load());
    status.add("board_status_crc_errors", status_crc_errors_.load());
    status.add("board_status_semantic_errors", status_semantic_errors_.load());
    status.add("board_status_age_ms", board_status_age_ms);
    status.add("board_runtime_frames", snapshot.runtime_generation);
    status.add("board_runtime_crc_errors", runtime_crc_errors_.load());
    status.add("board_cpu_idle_permille", snapshot.runtime.cpu_idle_permille);
    status.add("board_control_wcet_us", snapshot.runtime.control_wcet_us);
    status.add("board_uart_wcet_us", snapshot.runtime.uart_wcet_us);
    status.add("board_control_deadline_misses", snapshot.runtime.control_deadline_misses);
    status.add("board_uart_deadline_misses", snapshot.runtime.uart_deadline_misses);
    status.add("board_reset_events", board_reset_events_.load());
    status.add("session_rotations", session_rotations_.load());
    status.add("ack_fault_events", ack_fault_events_.load());
    status.add("invalid_command_events", invalid_command_events_.load());
    status.add("command_timeout_events", command_timeout_events_.load());
    status.add("session_id_host", snapshot.session_id);
    status.add("session_id_board", snapshot.board.session_id);
    status.add("board_protocol_version", snapshot.board.protocol_version);
    status.add("board_boot_challenge_acquired", snapshot.have_boot_challenge);
    status.add("board_boot_challenge", snapshot.boot_challenge);
    status.add("board_boot_id", snapshot.board.boot_id);
    status.add("board_reset_cause", snapshot.board.reset_cause);
    status.add("session_established", session_ok);
    status.add("last_sequence_dispatched", snapshot.last_dispatched);
    status.add("last_sequence_received", snapshot.board.received_sequence);
    status.add("last_sequence_applied", snapshot.board.applied_sequence);
    const auto ack_lag = snapshot.have_dispatched ?
      static_cast<std::uint32_t>(snapshot.last_dispatched - snapshot.board.applied_sequence) : 0U;
    status.add("applied_sequence_lag", ack_lag);
    status.add("board_command_age_ms", snapshot.board.command_age_ms);
    status.add("board_rx_crc_errors", snapshot.board.rx_crc_errors);
    status.add("board_safety_reason", snapshot.board.safety_reason);
    status.add("command_publisher_count", static_cast<std::int64_t>(snapshot.publisher_count));
    status.add("authority_endpoint_identified", snapshot.authority_endpoint);
    status.add("authority_epoch", snapshot.authority_epoch);
    status.add("command_age_ms", command_age_ms);
    status.add("command_timeout_latched", snapshot.command_timeout_latched);
    status.add("imu_gyro_calibration_requested", snapshot.imu_calibration_requested);
    status.add("imu_gyro_calibration_active",
      (snapshot.board.flags & kStatusFlagImuCalibrating) != 0U);
    status.add("imu_gyro_calibration_succeeded",
      (snapshot.board.flags & kStatusFlagImuCalibrationOk) != 0U);
    status.add("imu_gyro_calibration_failed", snapshot.imu_calibration_local_failed ||
      (snapshot.board.flags & kStatusFlagImuCalibrationFail) != 0U);
    status.add("external_imu_attitude_valid", imu_attitude_valid_.load());
    status.add(
      "imu_yaw_offset_deg",
      imu_yaw_offset_rad_.load(std::memory_order_relaxed) / kDegreesToRadians);
    status.add("imu_heading_alignment_pending", heading_alignment_pending_);
    status.add("command_source", snapshot.command_source);
    status.add("command_arm_generation", snapshot.arm_generation);
    status.add("authorized_arm_generation", snapshot.authorized_generation);
    status.add("arm_authorized", snapshot.arm_authorized);
    status.add("disarm_seen_after_session", snapshot.saw_disarmed);
    status.add("last_transition_reason", snapshot.transition_reason);
  }

  std::string port_, authority_node_name_, authority_node_namespace_;
  int baud_{}, span_us_{}, command_timeout_ms_{}, heartbeat_timeout_ms_{};
  double gyro_stddev_{}, accel_stddev_{};
  std::atomic<double> imu_yaw_offset_rad_{0.0};
  std::atomic<bool> imu_attitude_valid_{false};
  bool heading_alignment_pending_{true};
  std::int64_t heading_alignment_after_ros_ns_{0};
  std::uint64_t last_tag_map_generation_{0U};
  std::mutex heading_history_mutex_;
  std::deque<RawHeadingSample> raw_heading_history_;

  // These buffers/requests precede serial_ so they outlive cancellation of
  // any operation that references them during member destruction.
  boost::asio::io_context io_;
  std::array<std::uint8_t, 1024> read_chunk_{};
  std::shared_ptr<WriteRequest> active_write_;
  std::optional<WriteRequest> pending_write_;
  boost::asio::serial_port serial_;
  boost::asio::steady_timer reconnect_timer_io_;
  boost::asio::steady_timer write_deadline_io_;
  boost::asio::steady_timer shutdown_deadline_io_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
  std::thread io_thread_;
  std::uint64_t write_token_{0U};
  bool link_open_{false};
  bool shutdown_finished_io_{false};
  bool shutdown_neutral_queued_io_{false};
  std::uint32_t shutdown_session_id_io_{0U};
  std::uint32_t shutdown_sequence_io_{0U};
  std::uint64_t link_generation_io_{0U};
  std::shared_ptr<std::promise<void>> shutdown_promise_;

  std::vector<std::uint8_t> rx_buffer_;
  bool have_counter_{false}, have_mcu_tick_{false};
  std::uint32_t last_counter_{0U}, last_mcu_tick_{0U};
  McuClockMapper clock_mapper_;

  mutable std::mutex state_mutex_;
  BoardStatusFrame latest_status_{};
  RuntimeFrame latest_runtime_{};
  std::array<std::int16_t, kThrusterChannels> command_offsets_{};
  bool have_status_{false};
  bool have_runtime_{false};
  bool status_semantic_fault_{false};
  bool heartbeat_timeout_latched_{false};
  bool have_boot_identity_{false};
  bool have_identity_tick_{false};
  bool have_link_boot_challenge_{false};
  std::uint32_t boot_identity_{0U};
  std::uint32_t last_identity_tick_{0U};
  std::uint32_t link_boot_id_{0U};
  std::int64_t last_status_ns_{0};
  std::int64_t last_runtime_ns_{0};
  std::uint64_t runtime_generation_{0U};
  std::uint64_t published_runtime_generation_{0U};

  std::uint32_t session_id_{0U};
  std::uint32_t command_sequence_{0U};
  bool have_dispatched_sequence_{false};
  std::uint32_t last_dispatched_sequence_{0U};
  bool have_handshake_sequence_{false};
  std::uint32_t handshake_sequence_{0U};
  bool have_ack_baseline_{false};
  std::uint32_t last_ack_received_{0U};
  std::uint32_t last_ack_applied_{0U};
  bool session_acknowledged_{false};
  bool ack_fault_latched_{false};
  std::deque<DispatchedCommand> dispatched_history_;

  bool command_publisher_gate_{false};
  bool authority_publisher_identified_{false};
  std::uint64_t authority_epoch_{0U};
  std::size_t command_publisher_count_{0U};
  bool command_valid_{false};
  bool command_enabled_{false};
  bool command_armed_{false};
  std::uint64_t command_arm_generation_{0U};
  std::uint64_t authorized_generation_{0U};
  bool arm_authorized_{false};
  bool saw_disarmed_after_session_{false};
  std::uint64_t disarmed_generation_{0U};
  std::int64_t last_command_ns_{0};
  bool command_timeout_latched_{false};
  bool imu_calibration_requested_{false};
  bool imu_calibration_seen_active_{false};
  bool imu_calibration_local_failed_{false};
  std::int64_t imu_calibration_request_ns_{0};
  std::string command_source_;
  std::string transition_reason_;

  std::mutex outbound_mutex_;
  std::optional<WriteRequest> latest_outbound_;
  bool outbound_notification_posted_{false};

  boost::lockfree::spsc_queue<QueuedImu, boost::lockfree::capacity<256>> imu_queue_;
  std::thread imu_publish_thread_;
  std::mutex imu_queue_wait_mutex_;
  std::condition_variable imu_queue_cv_;
  std::mutex imu_statistics_mutex_;
  std::deque<std::int64_t> imu_publish_times_;
  std::deque<double> imu_transport_ms_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> imu_publish_stop_{false};
  std::atomic<bool> authority_refresh_requested_{true};
  std::atomic<std::uint64_t> link_generation_{0U};
  std::atomic<std::int64_t> last_imu_arrival_steady_ns_{0}, last_imu_stamp_ros_ns_{0};
  std::atomic<std::uint64_t> imu_frames_{0U}, imu_sequence_gaps_{0U}, imu_crc_errors_{0U};
  std::atomic<std::uint64_t> imu_duplicate_or_backwards_{0U}, imu_time_errors_{0U};
  std::atomic<std::uint64_t> ros_clock_discontinuities_{0U};
  std::atomic<std::uint64_t> imu_version_errors_{0U}, imu_invalid_flags_{0U};
  std::atomic<std::uint64_t> imu_queue_overflows_{0U}, imu_stale_generation_drops_{0U};
  std::atomic<std::uint64_t> status_frames_{0U}, status_crc_errors_{0U};
  std::atomic<std::uint64_t> runtime_crc_errors_{0U};
  std::atomic<std::uint64_t> status_semantic_errors_{0U}, board_reset_events_{0U};
  std::atomic<std::uint64_t> session_rotations_{0U}, ack_fault_events_{0U};
  std::atomic<std::uint64_t> invalid_command_events_{0U}, command_timeout_events_{0U};
  std::atomic<std::size_t> queue_high_water_{0U};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::BoardStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::BoardRuntime>::SharedPtr runtime_pub_;
  rclcpp::Subscription<robotcore_interfaces::msg::ThrusterCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<robotcore_interfaces::msg::AprilTagPoseEstimate>::SharedPtr tag_pose_sub_;
  rclcpp::Subscription<robotcore_interfaces::msg::BodyState>::SharedPtr body_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calibration_service_;
  rclcpp::TimerBase::SharedPtr command_timer_, status_timer_, diagnostic_timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace robotcore_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotcore_hardware::AboardBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
