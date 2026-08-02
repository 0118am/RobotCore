#include "eup_hardware/aboard_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace eup_hardware
{
namespace
{
constexpr double kGravity = 9.80665;
std::uint16_t read_u16(const std::uint8_t * p)
{
  return static_cast<std::uint16_t>(p[0]) | static_cast<std::uint16_t>(p[1]) << 8U;
}
std::uint32_t read_u32(const std::uint8_t * p)
{
  return static_cast<std::uint32_t>(p[0]) | static_cast<std::uint32_t>(p[1]) << 8U |
         static_cast<std::uint32_t>(p[2]) << 16U | static_cast<std::uint32_t>(p[3]) << 24U;
}
std::int16_t read_i16(const std::uint8_t * p)
{
  return static_cast<std::int16_t>(read_u16(p));
}
void append_i16(std::vector<std::uint8_t> & output, std::int16_t value)
{
  const auto word = static_cast<std::uint16_t>(value);
  output.push_back(static_cast<std::uint8_t>(word & 0xFFU));
  output.push_back(static_cast<std::uint8_t>(word >> 8U));
}
}  // namespace

std::uint16_t crc16_ccitt(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
    for (unsigned bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
        ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
        : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<std::uint8_t> build_direct_pwm_frame(
  const std::array<std::int16_t, kPwmChannels> & offsets)
{
  std::vector<std::uint8_t> frame;
  frame.reserve(kDirectPwmFrameSize);
  frame.push_back(0xFFU);
  frame.push_back(0xFAU);
  for (const auto value : offsets) {append_i16(frame, value);}
  std::uint8_t checksum = 0U;
  for (const auto value : frame) {checksum = static_cast<std::uint8_t>(checksum + value);}
  frame.push_back(checksum);
  return frame;
}

std::optional<std::array<std::uint16_t, kPwmChannels>> parse_pwm_feedback(
  const std::uint8_t * data, std::size_t size)
{
  if (size != kPwmFeedbackFrameSize || data[0] != 0xFFU || data[1] != 0xFBU) {
    return std::nullopt;
  }
  std::uint8_t checksum = 0U;
  for (std::size_t i = 0; i + 1U < size; ++i) {
    checksum = static_cast<std::uint8_t>(checksum + data[i]);
  }
  if (checksum != data[size - 1U]) {return std::nullopt;}
  std::array<std::uint16_t, kPwmChannels> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {result[i] = read_u16(data + 2U + 2U * i);}
  return result;
}

std::optional<ImuFrame> parse_imu_v1(const std::uint8_t * data, std::size_t size)
{
  if (size != kImuV1FrameSize || data[0] != 0xFFU || data[1] != 0xF8U ||
      data[2] != 0x04U || data[3] != 0x01U) {return std::nullopt;}
  if (read_u16(data + size - 2U) != crc16_ccitt(data, size - 2U)) {return std::nullopt;}
  ImuFrame result;
  result.version = data[3];
  result.flags = data[4];
  result.sample_counter = read_u32(data + 5U);
  result.sample_tick_ms = read_u32(data + 9U);
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  for (std::size_t i = 0; i < 3U; ++i) {
    result.gyro_rad_s[i] = static_cast<double>(read_i16(data + 13U + 2U * i)) * 0.01 * kDegToRad;
    result.accel_m_s2[i] = static_cast<double>(read_i16(data + 19U + 2U * i)) * 0.001 * kGravity;
  }
  return (result.flags & 0x01U) != 0U ? std::optional<ImuFrame>(result) : std::nullopt;
}

std::optional<std::int64_t> McuClockMapper::map(
  std::uint32_t tick_ms, std::int64_t arrival_ros_ns)
{
  if (!initialized_) {
    initialized_ = true;
    last_raw_tick_ = tick_ms;
    reference_tick_ = tick_ms;
    reference_arrival_ns_ = arrival_ros_ns;
    minimum_intercept_ns_ = static_cast<double>(arrival_ros_ns);
    last_stamp_ns_ = arrival_ros_ns;
    return arrival_ros_ns;
  }
  if (tick_ms < last_raw_tick_ && last_raw_tick_ - tick_ms > 0x80000000U) {
    wrap_epoch_ += (1ULL << 32U);
  } else if (tick_ms < last_raw_tick_) {
    return std::nullopt;
  }
  last_raw_tick_ = tick_ms;
  const std::uint64_t extended = wrap_epoch_ + tick_ms;
  const double elapsed_ticks = static_cast<double>(extended - reference_tick_);
  // Arrival time is the affine MCU clock plus a non-negative, varying queue
  // delay. Estimate its slope only over a long baseline, reject oscillator
  // outliers, and use a slow update so one delayed packet cannot move time.
  if (elapsed_ticks >= 1000.0) {
    const double observed_slope =
      static_cast<double>(arrival_ros_ns - reference_arrival_ns_) / elapsed_ticks;
    const double bounded_slope = std::clamp(observed_slope, 995000.0, 1005000.0);
    ns_per_tick_ = 0.98 * ns_per_tick_ + 0.02 * bounded_slope;
  }
  // The lower envelope rejects positive serial/executor delay and supplies the
  // intercept of the robust affine map t_ros = alpha + beta * t_mcu.
  const double candidate_intercept = static_cast<double>(arrival_ros_ns) - ns_per_tick_ * elapsed_ticks;
  minimum_intercept_ns_ = std::min(minimum_intercept_ns_, candidate_intercept);
  std::int64_t stamp = static_cast<std::int64_t>(
    std::llround(minimum_intercept_ns_ + ns_per_tick_ * elapsed_ticks));
  stamp = std::min(stamp, arrival_ros_ns);
  if (stamp <= last_stamp_ns_) {return std::nullopt;}
  last_stamp_ns_ = stamp;
  return stamp;
}

void McuClockMapper::reset() {*this = McuClockMapper{};}
}  // namespace eup_hardware
