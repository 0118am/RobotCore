#include "eup_hardware/aboard_protocol.hpp"

#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <eup_interfaces/msg/board_status.hpp>
#include <eup_interfaces/msg/thruster_command.hpp>
#include <eup_interfaces/msg/thruster_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace eup_hardware
{
class AboardBridgeNode final : public rclcpp::Node
{
public:
  AboardBridgeNode()
  : Node("aboard_bridge_node"), serial_(io_), work_(boost::asio::make_work_guard(io_)), updater_(this)
  {
    port_ = declare_parameter<std::string>(
      "serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A033320-if00");
    baud_ = declare_parameter<int>("baud", 115200);
    span_us_ = declare_parameter<int>("span_us", 100);
    channel_offset_ = declare_parameter<int>("thruster_channel_offset", 0);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 150);
    heartbeat_timeout_ms_ = declare_parameter<int>("heartbeat_timeout_ms", 250);
    const auto write_hz = declare_parameter<double>("command_write_hz", 100.0);
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "aboard_imu_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/hardware/aboard_imu_raw");
    gyro_stddev_ = declare_parameter<double>("imu_angular_velocity_stddev_rps", 0.05);
    accel_stddev_ = declare_parameter<double>("imu_linear_acceleration_stddev_mps2", 0.5);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS().keep_last(8));
    status_pub_ = create_publisher<eup_interfaces::msg::BoardStatus>("/hardware/board_status", 10);
    thruster_pub_ = create_publisher<eup_interfaces::msg::ThrusterState>("/robot/thruster_state", 10);
    command_sub_ = create_subscription<eup_interfaces::msg::ThrusterCommand>(
      "/control/thruster_cmd", 10,
      std::bind(&AboardBridgeNode::on_command, this, std::placeholders::_1));
    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, write_hz)),
      std::bind(&AboardBridgeNode::write_heartbeat, this));
    status_timer_ = create_wall_timer(100ms, std::bind(&AboardBridgeNode::publish_status, this));
    imu_drain_timer_ = create_wall_timer(1ms, std::bind(&AboardBridgeNode::drain_imu_queue, this));
    reconnect_timer_ = create_wall_timer(1s, std::bind(&AboardBridgeNode::ensure_open, this));
    updater_.setHardwareID("aboard-uart6");
    updater_.add("A-board serial and IMU", this, &AboardBridgeNode::diagnose);

    io_thread_ = std::thread([this]() {io_.run();});
    ensure_open();
  }

  ~AboardBridgeNode() override
  {
    std::array<std::int16_t, kPwmChannels> neutral{};
    write_frame(build_direct_pwm_frame(neutral));
    boost::system::error_code error;
    serial_.cancel(error);
    serial_.close(error);
    work_.reset();
    io_.stop();
    if (io_thread_.joinable()) {io_thread_.join();}
  }

private:
  struct QueuedImu
  {
    ImuFrame sample;
    std::int64_t stamp_ns{};
    std::int64_t arrival_ns{};
  };

  static std::int64_t steady_now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void ensure_open()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_.is_open()) {return;}
    boost::system::error_code error;
    serial_.open(port_, error);
    if (error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot open A-board %s: %s",
        port_.c_str(), error.message().c_str());
      return;
    }
    if (flock(serial_.native_handle(), LOCK_EX | LOCK_NB) != 0) {
      RCLCPP_ERROR(get_logger(), "A-board serial device is already owned: %s", port_.c_str());
      serial_.close(error);
      return;
    }
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_), error);
    serial_.set_option(boost::asio::serial_port_base::character_size(8), error);
    serial_.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none), error);
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one), error);
    if (error) {
      RCLCPP_ERROR(get_logger(), "Cannot configure A-board serial: %s", error.message().c_str());
      serial_.close(error);
      return;
    }
    clock_mapper_.reset();
    have_counter_ = false;
    have_mcu_tick_ = false;
    // A service restart does not stop the STM32 stream. Discard bytes queued
    // while no reader existed so the first clock anchor is a live sample.
    if (::tcflush(serial_.native_handle(), TCIFLUSH) != 0) {
      RCLCPP_WARN(get_logger(), "Failed to flush stale A-board input on open");
    }
    connected_.store(true);
    start_read();
    RCLCPP_INFO(get_logger(), "Opened A-board UART6 %s at %d baud", port_.c_str(), baud_);
  }

  void start_read()
  {
    serial_.async_read_some(boost::asio::buffer(read_chunk_),
      [this](const boost::system::error_code & error, std::size_t size) {
        if (error) {
          connected_.store(false);
          if (error != boost::asio::error::operation_aborted) {
            RCLCPP_ERROR(get_logger(), "A-board read failed: %s", error.message().c_str());
          }
          boost::system::error_code ignored;
          serial_.close(ignored);
          return;
        }
        {
          std::lock_guard<std::mutex> lock(rx_mutex_);
          rx_buffer_.insert(rx_buffer_.end(), read_chunk_.begin(), read_chunk_.begin() + size);
          parse_rx();
        }
        start_read();
      });
  }

  void parse_rx()
  {
    while (rx_buffer_.size() >= 3U) {
      auto begin = std::find(rx_buffer_.begin(), rx_buffer_.end(), 0xFFU);
      if (begin != rx_buffer_.begin()) {rx_buffer_.erase(rx_buffer_.begin(), begin);}
      if (rx_buffer_.size() < 3U) {return;}
      std::size_t length = 0U;
      if (rx_buffer_[1] == 0xFBU) {length = kPwmFeedbackFrameSize;}
      else if (rx_buffer_[1] == 0xF8U) {
        length = rx_buffer_[2] == 0x04U ? kImuV1FrameSize : kLegacyTelemetryFrameSize;
      } else {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }
      if (rx_buffer_.size() < length) {return;}
      bool accepted = false;
      if (rx_buffer_[1] == 0xFBU) {
        const auto feedback = parse_pwm_feedback(rx_buffer_.data(), length);
        if (feedback) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          feedback_pwm_ = *feedback;
          last_feedback_ns_ = steady_now_ns();
          ++feedback_frames_;
          accepted = true;
        }
      } else if (rx_buffer_[2] == 0x04U) {
        if (rx_buffer_[3] != 0x01U) {
          ++imu_version_errors_;
          accepted = true;
        } else if ((rx_buffer_[4] & 0x01U) == 0U) {
          ++imu_invalid_flags_;
          accepted = true;
        } else {
          const auto sample = parse_imu_v1(rx_buffer_.data(), length);
          if (sample) {accepted = handle_imu(*sample);}
          else {++imu_crc_errors_;}
        }
      } else {
        accepted = legacy_checksum_valid(rx_buffer_.data(), length);
        if (accepted) {++legacy_telemetry_frames_;}
      }
      if (accepted) {rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + length);}
      else {rx_buffer_.erase(rx_buffer_.begin());}
    }
    if (rx_buffer_.size() > 4096U) {rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 64);}
  }

  static bool legacy_checksum_valid(const std::uint8_t * data, std::size_t size)
  {
    std::uint8_t sum = 0U;
    for (std::size_t i = 0; i + 1U < size; ++i) {sum = static_cast<std::uint8_t>(sum + data[i]);}
    return sum == data[size - 1U];
  }

  bool handle_imu(const ImuFrame & sample)
  {
    if (have_mcu_tick_ && sample.sample_tick_ms < last_mcu_tick_ &&
      last_mcu_tick_ - sample.sample_tick_ms <= 0x80000000U)
    {
      // The UART bridge remains enumerated across an STM32 reset, so a board
      // reboot does not necessarily produce a serial disconnect.
      clock_mapper_.reset();
      have_counter_ = false;
    }
    last_mcu_tick_ = sample.sample_tick_ms;
    have_mcu_tick_ = true;
    if (have_counter_) {
      const auto delta = static_cast<std::uint32_t>(sample.sample_counter - last_counter_);
      if (delta == 0U || delta > 0x80000000U) {++imu_duplicate_or_backwards_; return true;}
      if (delta > 1U) {imu_sequence_gaps_ += delta - 1U;}
    }
    have_counter_ = true;
    last_counter_ = sample.sample_counter;
    const auto arrival = get_clock()->now().nanoseconds();
    const auto stamp = clock_mapper_.map(sample.sample_tick_ms, arrival);
    if (!stamp) {++imu_time_errors_; return true;}
    QueuedImu queued{sample, *stamp, arrival};
    if (!imu_queue_.push(queued)) {
      ++imu_queue_overflows_;
      return true;
    }
    queue_high_water_.store(std::max(queue_high_water_.load(), imu_queue_.read_available()));
    return true;
  }

  void drain_imu_queue()
  {
    QueuedImu queued;
    while (imu_queue_.pop(queued)) {publish_imu(queued);}
  }

  void publish_imu(const QueuedImu & queued)
  {
    const auto & sample = queued.sample;
    sensor_msgs::msg::Imu message;
    message.header.stamp = rclcpp::Time(queued.stamp_ns, RCL_ROS_TIME);
    message.header.frame_id = imu_frame_;
    message.orientation_covariance[0] = -1.0;
    message.angular_velocity.x = sample.gyro_rad_s[0];
    message.angular_velocity.y = sample.gyro_rad_s[1];
    message.angular_velocity.z = sample.gyro_rad_s[2];
    message.linear_acceleration.x = sample.accel_m_s2[0];
    message.linear_acceleration.y = sample.accel_m_s2[1];
    message.linear_acceleration.z = sample.accel_m_s2[2];
    message.angular_velocity_covariance[0] = gyro_stddev_ * gyro_stddev_;
    message.angular_velocity_covariance[4] = gyro_stddev_ * gyro_stddev_;
    message.angular_velocity_covariance[8] = gyro_stddev_ * gyro_stddev_;
    message.linear_acceleration_covariance[0] = accel_stddev_ * accel_stddev_;
    message.linear_acceleration_covariance[4] = accel_stddev_ * accel_stddev_;
    message.linear_acceleration_covariance[8] = accel_stddev_ * accel_stddev_;
    imu_pub_->publish(message);
    const auto published_at = get_clock()->now().nanoseconds();
    last_imu_arrival_ros_ns_.store(queued.arrival_ns);
    last_imu_stamp_ros_ns_.store(queued.stamp_ns);
    imu_publish_times_.push_back(published_at);
    imu_transport_ms_.push_back((published_at - queued.stamp_ns) * 1e-6);
    while (imu_publish_times_.size() > 500U) {imu_publish_times_.pop_front();}
    while (imu_transport_ms_.size() > 500U) {imu_transport_ms_.pop_front();}
    ++imu_frames_;
  }

  void on_command(const eup_interfaces::msg::ThrusterCommand::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_enabled_ = message->enable;
    command_offsets_.fill(0);
    if (message->enable) {
      for (std::size_t i = 0; i < message->normalized.size(); ++i) {
        const auto channel = static_cast<std::size_t>(std::clamp(channel_offset_, 0, 8)) + i;
        command_offsets_[channel] = static_cast<std::int16_t>(
          std::lround(std::clamp(static_cast<double>(message->normalized[i]), -1.0, 1.0) * span_us_));
      }
    }
    last_command_ns_ = steady_now_ns();
  }

  void write_heartbeat()
  {
    std::array<std::int16_t, kPwmChannels> output{};
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const auto age_ms = (steady_now_ns() - last_command_ns_) / 1000000LL;
      if (command_enabled_ && age_ms <= command_timeout_ms_) {output = command_offsets_;}
      else {command_enabled_ = false;}
    }
    write_frame(build_direct_pwm_frame(output));
  }

  void write_frame(const std::vector<std::uint8_t> & frame)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_.is_open()) {return;}
    boost::system::error_code error;
    boost::asio::write(serial_, boost::asio::buffer(frame), error);
    if (error) {
      connected_.store(false);
      RCLCPP_ERROR(get_logger(), "A-board write failed: %s", error.message().c_str());
      serial_.close(error);
    }
  }

  void publish_status()
  {
    const auto now = get_clock()->now();
    const auto steady = steady_now_ns();
    eup_interfaces::msg::BoardStatus status;
    status.header.stamp = now;
    status.connected = connected_.load();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      status.heartbeat_ok = last_feedback_ns_ > 0 &&
        (steady - last_feedback_ns_) / 1000000LL <= heartbeat_timeout_ms_;
      status.failsafe_active = !status.heartbeat_ok;
      status.estop_active = false;
      status.pwm_us.assign(feedback_pwm_.begin(), feedback_pwm_.end());
    }
    status.bus_voltage_v = std::numeric_limits<float>::quiet_NaN();
    status.board_temp_c = std::numeric_limits<float>::quiet_NaN();
    status.firmware_version = imu_frames_.load() > 0U ? "frame4-v1" :
      (legacy_telemetry_frames_.load() > 0U ? "legacy-telemetry" : "unknown; host expects frame4-v1");
    status_pub_->publish(status);

    eup_interfaces::msg::ThrusterState thrusters;
    thrusters.header = status.header;
    for (std::size_t i = 0; i < 8U; ++i) {
      const auto channel = static_cast<std::size_t>(std::clamp(channel_offset_, 0, 8)) + i;
      const auto pwm = status.pwm_us.size() > channel ? status.pwm_us[channel] : 1500U;
      thrusters.normalized_feedback[i] = static_cast<float>(static_cast<int>(pwm) - 1500) /
        static_cast<float>(std::max(1, span_us_));
      thrusters.pwm_us.push_back(pwm);
      thrusters.healthy.push_back(status.heartbeat_ok);
    }
    thruster_pub_->publish(thrusters);
    updater_.force_update();
  }

  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const auto now = get_clock()->now().nanoseconds();
    const auto last_arrival = last_imu_arrival_ros_ns_.load();
    const double imu_age_ms = last_arrival > 0 ? (now - last_arrival) * 1e-6 : INFINITY;
    const bool connected = connected_.load();
    const bool imu_fresh = imu_age_ms <= 50.0;
    const std::string summary = !connected ? "serial disconnected" :
      (imu_fresh ? "serial and frame-4 IMU healthy" :
      (legacy_telemetry_frames_.load() > 0U ? "legacy telemetry received; frame-4 IMU unavailable" :
      "serial connected; no frame-4 IMU telemetry"));
    status.summary(connected && imu_fresh ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN, summary);
    status.add("imu_received_count", imu_frames_.load());
    status.add("legacy_telemetry_frames", legacy_telemetry_frames_.load());
    status.add("imu_sequence_gaps", imu_sequence_gaps_.load());
    status.add("imu_crc_errors", imu_crc_errors_.load());
    status.add("imu_version_errors", imu_version_errors_.load());
    status.add("imu_invalid_flags", imu_invalid_flags_.load());
    status.add("imu_duplicate_or_backwards", imu_duplicate_or_backwards_.load());
    status.add("imu_time_errors", imu_time_errors_.load());
    status.add("imu_age_ms", imu_age_ms);
    const auto stamp = last_imu_stamp_ros_ns_.load();
    status.add("imu_transport_ms", stamp > 0 ? (now - stamp) * 1e-6 : INFINITY);
    double imu_rate_hz = 0.0;
    if (imu_publish_times_.size() >= 2U) {
      imu_rate_hz = static_cast<double>(imu_publish_times_.size() - 1U) * 1e9 /
        static_cast<double>(imu_publish_times_.back() - imu_publish_times_.front());
    }
    double p95_ms = INFINITY;
    if (!imu_transport_ms_.empty()) {
      auto sorted = std::vector<double>(imu_transport_ms_.begin(), imu_transport_ms_.end());
      const auto index = static_cast<std::size_t>(std::ceil(0.95 * sorted.size())) - 1U;
      std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
      p95_ms = sorted[index];
    }
    status.add("imu_rate_hz", imu_rate_hz);
    status.add("imu_transport_p95_ms", p95_ms);
    status.add("imu_queue_high_water", queue_high_water_.load());
    status.add("imu_queue_overflows", imu_queue_overflows_.load());
    status.add("feedback_frames", feedback_frames_.load());
  }

  std::string port_, imu_frame_, imu_topic_;
  int baud_{}, span_us_{}, channel_offset_{}, command_timeout_ms_{}, heartbeat_timeout_ms_{};
  double gyro_stddev_{}, accel_stddev_{};
  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
  std::thread io_thread_;
  std::array<std::uint8_t, 1024> read_chunk_{};
  std::vector<std::uint8_t> rx_buffer_;
  std::mutex serial_mutex_, rx_mutex_, command_mutex_, state_mutex_;
  std::array<std::int16_t, kPwmChannels> command_offsets_{};
  std::array<std::uint16_t, kPwmChannels> feedback_pwm_{};
  bool command_enabled_{false}, have_counter_{false}, have_mcu_tick_{false};
  std::uint32_t last_counter_{};
  std::uint32_t last_mcu_tick_{};
  std::int64_t last_command_ns_{}, last_feedback_ns_{};
  McuClockMapper clock_mapper_;
  boost::lockfree::spsc_queue<QueuedImu, boost::lockfree::capacity<256>> imu_queue_;
  std::deque<std::int64_t> imu_publish_times_;
  std::deque<double> imu_transport_ms_;
  std::atomic<bool> connected_{false};
  std::atomic<std::int64_t> last_imu_arrival_ros_ns_{0}, last_imu_stamp_ros_ns_{0};
  std::atomic<std::uint64_t> imu_frames_{0}, imu_sequence_gaps_{0}, imu_crc_errors_{0};
  std::atomic<std::uint64_t> imu_duplicate_or_backwards_{0}, imu_time_errors_{0}, feedback_frames_{0};
  std::atomic<std::uint64_t> imu_version_errors_{0}, imu_invalid_flags_{0}, imu_queue_overflows_{0};
  std::atomic<std::uint64_t> legacy_telemetry_frames_{0};
  std::atomic<std::size_t> queue_high_water_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<eup_interfaces::msg::BoardStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<eup_interfaces::msg::ThrusterState>::SharedPtr thruster_pub_;
  rclcpp::Subscription<eup_interfaces::msg::ThrusterCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr command_timer_, status_timer_, reconnect_timer_, imu_drain_timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace eup_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eup_hardware::AboardBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
