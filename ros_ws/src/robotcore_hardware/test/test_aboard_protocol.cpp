#include "robotcore_hardware/aboard_protocol.hpp"
#include <gtest/gtest.h>

namespace
{
void put_u16(std::uint8_t * destination, std::uint16_t value)
{
  destination[0] = static_cast<std::uint8_t>(value);
  destination[1] = static_cast<std::uint8_t>(value >> 8U);
}

void put_u32(std::uint8_t * destination, std::uint32_t value)
{
  destination[0] = static_cast<std::uint8_t>(value);
  destination[1] = static_cast<std::uint8_t>(value >> 8U);
  destination[2] = static_cast<std::uint8_t>(value >> 16U);
  destination[3] = static_cast<std::uint8_t>(value >> 24U);
}
}  // namespace

TEST(AboardProtocol, CrcGoldenVector)
{
  const std::uint8_t text[] = {'1','2','3','4','5','6','7','8','9'};
  EXPECT_EQ(robotcore_hardware::crc16_ccitt(text, sizeof(text)), 0x29B1U);
}

TEST(AboardProtocol, BuildsAndParsesV2Command)
{
  std::array<std::int16_t, robotcore_hardware::kThrusterChannels> offsets{
    -100, -1, 0, 1, 25, 50, 99, 100};
  auto frame = robotcore_hardware::build_command_v2(
    0x01020304U, 0x12345678U, 0x9ABCDEF0U, true, offsets);
  const std::array<std::uint8_t, robotcore_hardware::kCommandV2FrameSize> golden{
    0xff, 0xfc, 0x02, 0x01, 0x04, 0x03, 0x02, 0x01,
    0x78, 0x56, 0x34, 0x12, 0xf0, 0xde, 0xbc, 0x9a,
    0x9c, 0xff, 0xff, 0xff, 0x00, 0x00, 0x01, 0x00,
    0x19, 0x00, 0x32, 0x00, 0x63, 0x00, 0x64, 0x00,
    0xc3, 0xdc};

  ASSERT_EQ(frame.size(), robotcore_hardware::kCommandV2FrameSize);
  EXPECT_EQ(frame, golden);
  EXPECT_EQ(frame[0], robotcore_hardware::kFrameHead);
  EXPECT_EQ(frame[1], robotcore_hardware::kCommandV2Type);
  const auto parsed = robotcore_hardware::parse_command_v2(frame.data(), frame.size());
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->flags, robotcore_hardware::kCommandFlagEnable);
  EXPECT_EQ(parsed->boot_id, 0x01020304U);
  EXPECT_EQ(parsed->session_id, 0x12345678U);
  EXPECT_EQ(parsed->sequence, 0x9ABCDEF0U);
  EXPECT_EQ(parsed->offsets_us, offsets);

  frame[16] ^= 0x01U;
  EXPECT_FALSE(robotcore_hardware::parse_command_v2(frame.data(), frame.size()));
}

TEST(AboardProtocol, ParsesV2BoardStatus)
{
  std::array<std::uint8_t, robotcore_hardware::kStatusV2FrameSize> frame{};
  frame[0] = robotcore_hardware::kFrameHead;
  frame[1] = robotcore_hardware::kStatusV2Type;
  frame[2] = robotcore_hardware::kProtocolV2;
  frame[3] = robotcore_hardware::kStatusFlagSessionEstablished |
    robotcore_hardware::kStatusFlagOutputsEnabled;
  put_u32(frame.data() + 4U, 4321U);
  put_u32(frame.data() + 8U, 0x01020304U);
  put_u32(frame.data() + 12U, 0xAABBCCDDU);
  put_u32(frame.data() + 16U, 101U);
  put_u32(frame.data() + 20U, 100U);
  put_u16(frame.data() + 24U, 17U);
  frame[26] = 3U;
  frame[27] = 0xA5U;
  put_u16(frame.data() + 28U, 7U);
  for (std::size_t i = 0; i < robotcore_hardware::kThrusterChannels; ++i) {
    put_u16(frame.data() + 30U + 2U * i, static_cast<std::uint16_t>(1500U + i));
  }
  put_u16(
    frame.data() + frame.size() - 2U,
    robotcore_hardware::crc16_ccitt(frame.data(), frame.size() - 2U));

  const std::array<std::uint8_t, robotcore_hardware::kStatusV2FrameSize> golden{
    0xff, 0xfd, 0x02, 0x03, 0xe1, 0x10, 0x00, 0x00,
    0x04, 0x03, 0x02, 0x01, 0xdd, 0xcc, 0xbb, 0xaa,
    0x65, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
    0x11, 0x00, 0x03, 0xa5, 0x07, 0x00, 0xdc, 0x05,
    0xdd, 0x05, 0xde, 0x05, 0xdf, 0x05, 0xe0, 0x05,
    0xe1, 0x05, 0xe2, 0x05, 0xe3, 0x05, 0x3b, 0x20};
  EXPECT_EQ(frame, golden);

  const auto parsed = robotcore_hardware::parse_board_status_v2(frame.data(), frame.size());
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->protocol_version, robotcore_hardware::kProtocolV2);
  EXPECT_EQ(parsed->board_tick_ms, 4321U);
  EXPECT_EQ(parsed->boot_id, 0x01020304U);
  EXPECT_EQ(parsed->session_id, 0xAABBCCDDU);
  EXPECT_EQ(parsed->received_sequence, 101U);
  EXPECT_EQ(parsed->applied_sequence, 100U);
  EXPECT_EQ(parsed->command_age_ms, 17U);
  EXPECT_EQ(parsed->safety_reason, 3U);
  EXPECT_EQ(parsed->reset_cause, 0xA5U);
  EXPECT_EQ(parsed->rx_crc_errors, 7U);
  EXPECT_EQ(parsed->pwm_us.front(), 1500U);
  EXPECT_EQ(parsed->pwm_us.back(), 1507U);

  frame[3] |= 0x08U;
  put_u16(
    frame.data() + frame.size() - 2U,
    robotcore_hardware::crc16_ccitt(frame.data(), frame.size() - 2U));
  EXPECT_FALSE(robotcore_hardware::parse_board_status_v2(frame.data(), frame.size()));
}

TEST(AboardProtocol, V2FitsThe115200BaudJitterBudget)
{
  constexpr double baud = 115200.0;
  constexpr double serial_bytes_per_second = baud / 10.0;  // 8N1
  constexpr double command_bytes_per_second = robotcore_hardware::kCommandV2FrameSize * 50.0;
  constexpr double telemetry_bytes_per_second =
    robotcore_hardware::kImuV1FrameSize * 100.0 + robotcore_hardware::kStatusV2FrameSize * 20.0;
  constexpr double coincident_telemetry_burst_ms =
    (robotcore_hardware::kImuV1FrameSize + robotcore_hardware::kStatusV2FrameSize) * 10.0 * 1000.0 /
    baud;

  EXPECT_LT(command_bytes_per_second / serial_bytes_per_second, 0.15);
  EXPECT_LT(telemetry_bytes_per_second / serial_bytes_per_second, 0.32);
  EXPECT_LT(coincident_telemetry_burst_ms, 7.0);
}

TEST(AboardProtocol, SafetyReasonAndFlagsMustDescribeOneState)
{
  robotcore_hardware::BoardStatusFrame status;

  status.safety_reason = robotcore_hardware::kSafetyReasonOk;
  status.flags = robotcore_hardware::kStatusFlagSessionEstablished |
    robotcore_hardware::kStatusFlagOutputsEnabled;
  EXPECT_TRUE(robotcore_hardware::board_status_reason_flags_consistent(status));

  status.safety_reason = robotcore_hardware::kSafetyReasonDisabled;
  EXPECT_TRUE(robotcore_hardware::board_status_reason_flags_consistent(status));

  status.safety_reason = 2U;  // command timeout
  status.flags = robotcore_hardware::kStatusFlagFailsafe |
    robotcore_hardware::kStatusFlagOutputsEnabled;
  EXPECT_TRUE(robotcore_hardware::board_status_reason_flags_consistent(status));

  status.flags = robotcore_hardware::kStatusFlagSessionEstablished;
  EXPECT_FALSE(robotcore_hardware::board_status_reason_flags_consistent(status));
  status.safety_reason = robotcore_hardware::kSafetyReasonOk;
  status.flags = robotcore_hardware::kStatusFlagFailsafe;
  EXPECT_FALSE(robotcore_hardware::board_status_reason_flags_consistent(status));
  status.safety_reason = robotcore_hardware::kMaximumSafetyReason + 1U;
  EXPECT_FALSE(robotcore_hardware::board_status_reason_flags_consistent(status));
}

TEST(AboardProtocol, ParsesFrame4)
{
  std::array<std::uint8_t, robotcore_hardware::kImuV1FrameSize> frame{};
  frame[0] = 0xFF; frame[1] = 0xF8; frame[2] = 4; frame[3] = 1; frame[4] = 1;
  frame[5] = 0x78; frame[6] = 0x56; frame[7] = 0x34; frame[8] = 0x12;
  frame[9] = 0xE8; frame[10] = 0x03;
  frame[13] = 100;  // 1 deg/s
  frame[23] = 0xE8; frame[24] = 0x03;  // 1 g
  const auto crc = robotcore_hardware::crc16_ccitt(frame.data(), frame.size() - 2);
  frame[25] = static_cast<std::uint8_t>(crc); frame[26] = static_cast<std::uint8_t>(crc >> 8);
  const auto parsed = robotcore_hardware::parse_imu_v1(frame.data(), frame.size());
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->sample_counter, 0x12345678U);
  EXPECT_NEAR(parsed->gyro_rad_s[0], 3.141592653589793 / 180.0, 1e-9);
  EXPECT_NEAR(parsed->accel_m_s2[2], 9.80665, 1e-9);
}

TEST(AboardProtocol, RejectsBadCrcAndBackwardsClock)
{
  std::array<std::uint8_t, robotcore_hardware::kImuV1FrameSize> frame{};
  frame[0] = 0xFF; frame[1] = 0xF8; frame[2] = 4; frame[3] = 1; frame[4] = 1;
  EXPECT_FALSE(robotcore_hardware::parse_imu_v1(frame.data(), frame.size()));
  robotcore_hardware::McuClockMapper mapper;
  ASSERT_TRUE(mapper.map(100, 1000000000LL));
  EXPECT_FALSE(mapper.map(99, 1010000000LL));
}

TEST(AboardProtocol, ExtendsThirtyTwoBitMcuClockWrap)
{
  robotcore_hardware::McuClockMapper mapper;
  const auto before = mapper.map(0xFFFFFFF0U, 1000000000LL);
  const auto after = mapper.map(5U, 1021000000LL);
  ASSERT_TRUE(before);
  ASSERT_TRUE(after);
  EXPECT_EQ(*after - *before, 21000000LL);
}

TEST(AboardProtocol, RejectsUnsupportedFrameVersion)
{
  std::array<std::uint8_t, robotcore_hardware::kImuV1FrameSize> frame{};
  frame[0] = 0xFF; frame[1] = 0xF8; frame[2] = 4; frame[3] = 2; frame[4] = 1;
  const auto crc = robotcore_hardware::crc16_ccitt(frame.data(), frame.size() - 2);
  frame[25] = static_cast<std::uint8_t>(crc); frame[26] = static_cast<std::uint8_t>(crc >> 8);
  EXPECT_FALSE(robotcore_hardware::parse_imu_v1(frame.data(), frame.size()));
}

TEST(AboardProtocol, AffineClockMappingRejectsPositiveArrivalJitter)
{
  robotcore_hardware::McuClockMapper mapper;
  const auto first = mapper.map(1000U, 2000000000LL);
  const auto delayed = mapper.map(2000U, 3005000000LL);
  const auto low_delay = mapper.map(3000U, 4001000000LL);
  ASSERT_TRUE(first);
  ASSERT_TRUE(delayed);
  ASSERT_TRUE(low_delay);
  EXPECT_LE(*delayed, 3005000000LL);
  EXPECT_LE(*low_delay, 4001000000LL);
  EXPECT_GT(*low_delay, *delayed);
}
