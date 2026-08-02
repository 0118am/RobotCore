#include "eup_hardware/aboard_protocol.hpp"
#include <gtest/gtest.h>

TEST(AboardProtocol, CrcGoldenVector)
{
  const std::uint8_t text[] = {'1','2','3','4','5','6','7','8','9'};
  EXPECT_EQ(eup_hardware::crc16_ccitt(text, sizeof(text)), 0x29B1U);
}

TEST(AboardProtocol, ParsesFrame4)
{
  std::array<std::uint8_t, eup_hardware::kImuV1FrameSize> frame{};
  frame[0] = 0xFF; frame[1] = 0xF8; frame[2] = 4; frame[3] = 1; frame[4] = 1;
  frame[5] = 0x78; frame[6] = 0x56; frame[7] = 0x34; frame[8] = 0x12;
  frame[9] = 0xE8; frame[10] = 0x03;
  frame[13] = 100;  // 1 deg/s
  frame[23] = 0xE8; frame[24] = 0x03;  // 1 g
  const auto crc = eup_hardware::crc16_ccitt(frame.data(), frame.size() - 2);
  frame[25] = static_cast<std::uint8_t>(crc); frame[26] = static_cast<std::uint8_t>(crc >> 8);
  const auto parsed = eup_hardware::parse_imu_v1(frame.data(), frame.size());
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->sample_counter, 0x12345678U);
  EXPECT_NEAR(parsed->gyro_rad_s[0], 3.141592653589793 / 180.0, 1e-9);
  EXPECT_NEAR(parsed->accel_m_s2[2], 9.80665, 1e-9);
}

TEST(AboardProtocol, RejectsBadCrcAndBackwardsClock)
{
  std::array<std::uint8_t, eup_hardware::kImuV1FrameSize> frame{};
  frame[0] = 0xFF; frame[1] = 0xF8; frame[2] = 4; frame[3] = 1; frame[4] = 1;
  EXPECT_FALSE(eup_hardware::parse_imu_v1(frame.data(), frame.size()));
  eup_hardware::McuClockMapper mapper;
  ASSERT_TRUE(mapper.map(100, 1000000000LL));
  EXPECT_FALSE(mapper.map(99, 1010000000LL));
}

TEST(AboardProtocol, ExtendsThirtyTwoBitMcuClockWrap)
{
  eup_hardware::McuClockMapper mapper;
  const auto before = mapper.map(0xFFFFFFF0U, 1000000000LL);
  const auto after = mapper.map(5U, 1021000000LL);
  ASSERT_TRUE(before);
  ASSERT_TRUE(after);
  EXPECT_EQ(*after - *before, 21000000LL);
}

TEST(AboardProtocol, RejectsUnsupportedFrameVersion)
{
  std::array<std::uint8_t, eup_hardware::kImuV1FrameSize> frame{};
  frame[0] = 0xFF; frame[1] = 0xF8; frame[2] = 4; frame[3] = 2; frame[4] = 1;
  const auto crc = eup_hardware::crc16_ccitt(frame.data(), frame.size() - 2);
  frame[25] = static_cast<std::uint8_t>(crc); frame[26] = static_cast<std::uint8_t>(crc >> 8);
  EXPECT_FALSE(eup_hardware::parse_imu_v1(frame.data(), frame.size()));
}

TEST(AboardProtocol, AffineClockMappingRejectsPositiveArrivalJitter)
{
  eup_hardware::McuClockMapper mapper;
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
