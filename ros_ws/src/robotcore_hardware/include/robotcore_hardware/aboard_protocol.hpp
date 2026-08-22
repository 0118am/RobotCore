#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace robotcore_hardware
{
constexpr std::uint8_t kFrameHead = 0xFFU;
constexpr std::uint8_t kCommandV2Type = 0xFCU;
constexpr std::uint8_t kStatusV2Type = 0xFDU;
constexpr std::uint8_t kProtocolV2 = 0x02U;
constexpr std::uint8_t kCommandFlagEnable = 0x01U;
constexpr std::uint8_t kCommandFlagImuGyroCalibrate = 0x02U;
constexpr std::uint8_t kStatusFlagSessionEstablished = 0x01U;
constexpr std::uint8_t kStatusFlagOutputsEnabled = 0x02U;
constexpr std::uint8_t kStatusFlagFailsafe = 0x04U;
constexpr std::uint8_t kStatusFlagImuCalibrating = 0x08U;
constexpr std::uint8_t kStatusFlagImuCalibrationOk = 0x10U;
constexpr std::uint8_t kStatusFlagImuCalibrationFail = 0x20U;
constexpr std::uint8_t kStatusFlagMask =
  kStatusFlagSessionEstablished | kStatusFlagOutputsEnabled |
  kStatusFlagFailsafe | kStatusFlagImuCalibrating |
  kStatusFlagImuCalibrationOk | kStatusFlagImuCalibrationFail;
constexpr std::uint8_t kSafetyReasonOk = 0U;
constexpr std::uint8_t kSafetyReasonDisabled = 3U;
constexpr std::uint8_t kMaximumSafetyReason = 6U;

constexpr std::size_t kThrusterChannels = 8;
constexpr std::size_t kCommandV2FrameSize = 34;
constexpr std::size_t kStatusV2FrameSize = 48;
constexpr std::size_t kImuV2FrameSize = 33;
constexpr std::uint8_t kImuFrameNumber = 0x04U;
constexpr std::uint8_t kImuFlagValid = 0x01U;
constexpr std::uint8_t kImuFlagAttitudeValid = 0x02U;
constexpr std::uint8_t kRuntimeFrameNumber = 0x05U;
constexpr std::size_t kRuntimeV1FrameSize = 37;

struct CommandFrame
{
  std::uint8_t flags{};
  std::uint32_t boot_id{};
  std::uint32_t session_id{};
  std::uint32_t sequence{};
  std::array<std::int16_t, kThrusterChannels> offsets_us{};
};

struct BoardStatusFrame
{
  std::uint8_t protocol_version{};
  std::uint8_t flags{};
  std::uint32_t board_tick_ms{};
  std::uint32_t boot_id{};
  std::uint32_t session_id{};
  std::uint32_t received_sequence{};
  std::uint32_t applied_sequence{};
  std::uint16_t command_age_ms{};
  std::uint8_t safety_reason{};
  std::uint8_t reset_cause{};
  std::uint16_t rx_crc_errors{};
  std::array<std::uint16_t, kThrusterChannels> pwm_us{};
};

struct ImuFrame
{
  std::uint8_t version{};
  std::uint8_t flags{};
  std::uint32_t sample_counter{};
  std::uint32_t sample_tick_ms{};
  std::array<double, 3> gyro_rad_s{};
  std::array<double, 3> accel_m_s2{};
  std::array<double, 3> attitude_rpy_rad{};
  bool attitude_valid{false};
};

struct RuntimeFrame
{
  std::uint32_t board_tick_ms{};
  std::uint16_t cpu_idle_permille{};
  std::uint16_t control_wcet_us{};
  std::uint16_t uart_wcet_us{};
  std::uint32_t control_deadline_misses{};
  std::uint32_t uart_deadline_misses{};
  std::uint16_t control_min_stack_words{};
  std::uint16_t uart_min_stack_words{};
  std::uint16_t stack_overflow_count{};
  std::uint16_t watchdog_missed_windows{};
  std::uint16_t uart_rx_dma_errors{};
  std::uint16_t uart_tx_drops{};
};

std::uint16_t crc16_ccitt(const std::uint8_t * data, std::size_t size);
std::array<std::uint8_t, kCommandV2FrameSize> build_command_v2(
  std::uint32_t boot_id, std::uint32_t session_id, std::uint32_t sequence, bool enable,
  const std::array<std::int16_t, kThrusterChannels> & offsets_us,
  bool calibrate_imu_gyro = false);
std::optional<CommandFrame> parse_command_v2(const std::uint8_t * data, std::size_t size);
std::optional<BoardStatusFrame> parse_board_status_v2(
  const std::uint8_t * data, std::size_t size);
bool board_status_matches_applied_command(
  const BoardStatusFrame & status, const CommandFrame & command);
bool board_status_reason_flags_consistent(const BoardStatusFrame & status);
std::optional<ImuFrame> parse_imu_v2(const std::uint8_t * data, std::size_t size);
std::array<double, 3> imu_vector_to_base_link(
  const std::array<double, 3> & sensor_vector);
std::array<double, 3> imu_attitude_rpy_to_base_link(
  const std::array<double, 3> & sensor_rpy);
std::optional<RuntimeFrame> parse_runtime_v1(const std::uint8_t * data, std::size_t size);

class McuClockMapper
{
public:
  std::optional<std::int64_t> map(
    std::uint32_t tick_ms, std::int64_t arrival_ros_ns,
    std::int64_t arrival_steady_ns);
  bool take_ros_clock_discontinuity();
  void reset();

private:
  bool initialized_{false};
  bool ros_clock_discontinuity_{false};
  std::uint32_t last_raw_tick_{};
  std::uint64_t wrap_epoch_{};
  std::uint64_t reference_tick_{};
  std::int64_t reference_arrival_steady_ns_{};
  std::int64_t last_arrival_ros_ns_{};
  std::int64_t last_arrival_steady_ns_{};
  double ns_per_tick_{1000000.0};
  double minimum_steady_intercept_ns_{};
  std::int64_t last_stamp_ns_{};
};
}  // namespace robotcore_hardware
