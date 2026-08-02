#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace eup_hardware
{
constexpr std::size_t kPwmChannels = 16;
constexpr std::size_t kDirectPwmFrameSize = 35;
constexpr std::size_t kPwmFeedbackFrameSize = 35;
constexpr std::size_t kLegacyTelemetryFrameSize = 20;
constexpr std::size_t kImuV1FrameSize = 27;

struct ImuFrame
{
  std::uint8_t version{};
  std::uint8_t flags{};
  std::uint32_t sample_counter{};
  std::uint32_t sample_tick_ms{};
  std::array<double, 3> gyro_rad_s{};
  std::array<double, 3> accel_m_s2{};
};

std::uint16_t crc16_ccitt(const std::uint8_t * data, std::size_t size);
std::vector<std::uint8_t> build_direct_pwm_frame(
  const std::array<std::int16_t, kPwmChannels> & offsets);
std::optional<std::array<std::uint16_t, kPwmChannels>> parse_pwm_feedback(
  const std::uint8_t * data, std::size_t size);
std::optional<ImuFrame> parse_imu_v1(const std::uint8_t * data, std::size_t size);

class McuClockMapper
{
public:
  std::optional<std::int64_t> map(std::uint32_t tick_ms, std::int64_t arrival_ros_ns);
  void reset();

private:
  bool initialized_{false};
  std::uint32_t last_raw_tick_{};
  std::uint64_t wrap_epoch_{};
  std::uint64_t reference_tick_{};
  std::int64_t reference_arrival_ns_{};
  double ns_per_tick_{1000000.0};
  double minimum_intercept_ns_{};
  std::int64_t last_stamp_ns_{};
};
}  // namespace eup_hardware
