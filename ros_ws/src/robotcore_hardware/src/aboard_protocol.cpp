#include "robotcore_hardware/aboard_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace robotcore_hardware
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
void write_u16(std::uint8_t * p, std::uint16_t value)
{
  p[0] = static_cast<std::uint8_t>(value & 0xFFU);
  p[1] = static_cast<std::uint8_t>(value >> 8U);
}
void write_u32(std::uint8_t * p, std::uint32_t value)
{
  p[0] = static_cast<std::uint8_t>(value & 0xFFU);
  p[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  p[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  p[3] = static_cast<std::uint8_t>(value >> 24U);
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

std::array<std::uint8_t, kCommandV2FrameSize> build_command_v2(
  std::uint32_t boot_id, std::uint32_t session_id, std::uint32_t sequence, bool enable,
  const std::array<std::int16_t, kThrusterChannels> & offsets_us)
{
  std::array<std::uint8_t, kCommandV2FrameSize> frame{};
  frame[0] = kFrameHead;
  frame[1] = kCommandV2Type;
  frame[2] = kProtocolV2;
  frame[3] = enable ? kCommandFlagEnable : 0U;
  write_u32(frame.data() + 4U, boot_id);
  write_u32(frame.data() + 8U, session_id);
  write_u32(frame.data() + 12U, sequence);
  for (std::size_t i = 0; i < offsets_us.size(); ++i) {
    write_u16(frame.data() + 16U + 2U * i, static_cast<std::uint16_t>(offsets_us[i]));
  }
  write_u16(frame.data() + frame.size() - 2U, crc16_ccitt(frame.data(), frame.size() - 2U));
  return frame;
}

std::optional<CommandFrame> parse_command_v2(const std::uint8_t * data, std::size_t size)
{
  if (size != kCommandV2FrameSize || data[0] != kFrameHead || data[1] != kCommandV2Type ||
      data[2] != kProtocolV2 || (data[3] & ~kCommandFlagEnable) != 0U ||
      read_u16(data + size - 2U) != crc16_ccitt(data, size - 2U))
  {
    return std::nullopt;
  }

  CommandFrame result;
  result.flags = data[3];
  result.boot_id = read_u32(data + 4U);
  result.session_id = read_u32(data + 8U);
  result.sequence = read_u32(data + 12U);
  for (std::size_t i = 0; i < result.offsets_us.size(); ++i) {
    result.offsets_us[i] = read_i16(data + 16U + 2U * i);
  }
  return result;
}

std::optional<BoardStatusFrame> parse_board_status_v2(
  const std::uint8_t * data, std::size_t size)
{
  if (size != kStatusV2FrameSize || data[0] != kFrameHead || data[1] != kStatusV2Type ||
      data[2] != kProtocolV2 || (data[3] & ~kStatusFlagMask) != 0U ||
      read_u16(data + size - 2U) != crc16_ccitt(data, size - 2U))
  {
    return std::nullopt;
  }

  BoardStatusFrame result;
  result.protocol_version = data[2];
  result.flags = data[3];
  result.board_tick_ms = read_u32(data + 4U);
  result.boot_id = read_u32(data + 8U);
  result.session_id = read_u32(data + 12U);
  result.received_sequence = read_u32(data + 16U);
  result.applied_sequence = read_u32(data + 20U);
  result.command_age_ms = read_u16(data + 24U);
  result.safety_reason = data[26];
  result.reset_cause = data[27];
  result.rx_crc_errors = read_u16(data + 28U);
  for (std::size_t i = 0; i < result.pwm_us.size(); ++i) {
    result.pwm_us[i] = read_u16(data + 30U + 2U * i);
  }
  return result;
}

bool board_status_matches_applied_command(
  const BoardStatusFrame & status, const CommandFrame & command)
{
  if (status.boot_id != command.boot_id || status.session_id != command.session_id ||
    status.applied_sequence != command.sequence)
  {
    return false;
  }
  const bool enabled = (command.flags & kCommandFlagEnable) != 0U;
  const bool outputs_enabled = (status.flags & kStatusFlagOutputsEnabled) != 0U;
  if (outputs_enabled != enabled) {return false;}
  for (std::size_t i = 0; i < command.offsets_us.size(); ++i) {
    const auto expected = static_cast<std::uint16_t>(
      enabled ? 1500 + command.offsets_us[i] : 1500);
    if (status.pwm_us[i] != expected) {return false;}
  }
  return true;
}

bool board_status_reason_flags_consistent(const BoardStatusFrame & status)
{
  if (status.safety_reason > kMaximumSafetyReason) {return false;}
  const bool session_established =
    (status.flags & kStatusFlagSessionEstablished) != 0U;
  const bool failsafe = (status.flags & kStatusFlagFailsafe) != 0U;
  const bool nominal_reason = status.safety_reason == kSafetyReasonOk ||
    status.safety_reason == kSafetyReasonDisabled;

  // Physical outputs may remain enabled for one PWM boundary while a fault
  // stages neutral, so outputs_enabled deliberately does not participate.
  return nominal_reason ? (session_established && !failsafe) :
         (!session_established && failsafe);
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

std::optional<RuntimeFrame> parse_runtime_v1(const std::uint8_t * data, std::size_t size)
{
  if (size != kRuntimeV1FrameSize || data[0] != kFrameHead || data[1] != 0xF8U ||
    data[2] != kRuntimeFrameNumber || data[3] != 0x01U || (data[4] & 0x01U) == 0U ||
    read_u16(data + size - 2U) != crc16_ccitt(data, size - 2U))
  {
    return std::nullopt;
  }
  RuntimeFrame result;
  result.board_tick_ms = read_u32(data + 5U);
  result.cpu_idle_permille = read_u16(data + 9U);
  result.control_wcet_us = read_u16(data + 11U);
  result.uart_wcet_us = read_u16(data + 13U);
  result.control_deadline_misses = read_u32(data + 15U);
  result.uart_deadline_misses = read_u32(data + 19U);
  result.control_min_stack_words = read_u16(data + 23U);
  result.uart_min_stack_words = read_u16(data + 25U);
  result.stack_overflow_count = read_u16(data + 27U);
  result.watchdog_missed_windows = read_u16(data + 29U);
  result.uart_rx_dma_errors = read_u16(data + 31U);
  result.uart_tx_drops = read_u16(data + 33U);
  return result;
}

std::optional<std::int64_t> McuClockMapper::map(
  std::uint32_t tick_ms, std::int64_t arrival_ros_ns,
  std::int64_t arrival_steady_ns)
{
  constexpr std::int64_t kRosClockJumpThresholdNs = 100000000LL;
  if (!initialized_) {
    initialized_ = true;
    last_raw_tick_ = tick_ms;
    reference_tick_ = tick_ms;
    reference_arrival_steady_ns_ = arrival_steady_ns;
    last_arrival_ros_ns_ = arrival_ros_ns;
    last_arrival_steady_ns_ = arrival_steady_ns;
    minimum_steady_intercept_ns_ = static_cast<double>(arrival_steady_ns);
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
  const auto ros_delta = arrival_ros_ns - last_arrival_ros_ns_;
  const auto steady_delta = arrival_steady_ns - last_arrival_steady_ns_;
  const bool ros_clock_jump =
    std::llabs(ros_delta - steady_delta) > kRosClockJumpThresholdNs;
  last_arrival_ros_ns_ = arrival_ros_ns;
  last_arrival_steady_ns_ = arrival_steady_ns;

  // Estimate oscillator slope exclusively against a monotonic clock. NTP or
  // an operator may step CLOCK_REALTIME while the bridge is running; using
  // ROS/system time here would turn that step into a permanent IMU age error.
  if (elapsed_ticks >= 1000.0) {
    const double observed_slope =
      static_cast<double>(arrival_steady_ns - reference_arrival_steady_ns_) /
      elapsed_ticks;
    const double bounded_slope = std::clamp(observed_slope, 995000.0, 1005000.0);
    ns_per_tick_ = 0.98 * ns_per_tick_ + 0.02 * bounded_slope;
  }

  // Find sample time in the monotonic domain using the lower arrival envelope,
  // then translate it with the current ROS-minus-monotonic offset. This keeps
  // serial/executor delay out of the stamp while allowing a legitimate wall
  // clock correction to take effect immediately.
  const double candidate_intercept =
    static_cast<double>(arrival_steady_ns) - ns_per_tick_ * elapsed_ticks;
  minimum_steady_intercept_ns_ =
    std::min(minimum_steady_intercept_ns_, candidate_intercept);
  const double ros_minus_steady =
    static_cast<double>(arrival_ros_ns - arrival_steady_ns);
  std::int64_t stamp = static_cast<std::int64_t>(
    std::llround(
      minimum_steady_intercept_ns_ + ns_per_tick_ * elapsed_ticks +
      ros_minus_steady));
  stamp = std::min(stamp, arrival_ros_ns);
  if (!ros_clock_jump && stamp <= last_stamp_ns_) {return std::nullopt;}
  ros_clock_discontinuity_ = ros_clock_discontinuity_ || ros_clock_jump;
  last_stamp_ns_ = stamp;
  return stamp;
}

bool McuClockMapper::take_ros_clock_discontinuity()
{
  const bool result = ros_clock_discontinuity_;
  ros_clock_discontinuity_ = false;
  return result;
}

void McuClockMapper::reset() {*this = McuClockMapper{};}
}  // namespace robotcore_hardware
