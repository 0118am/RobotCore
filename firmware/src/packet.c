#include "eup_firmware/packet.h"

uint16_t eup_crc16_ccitt(const uint8_t *data, size_t length) {
  /* Must match src/eup_hardware/eup_hardware/packet.py exactly. */
  uint16_t crc = 0xFFFFu;
  for (size_t index = 0; index < length; ++index) {
    crc ^= (uint16_t)data[index] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

uint16_t eup_normalized_to_pwm(float value) {
  /* Defensive clamp keeps out-of-range host commands inside the PWM window. */
  if (value > 1.0f) {
    value = 1.0f;
  }
  if (value < -1.0f) {
    value = -1.0f;
  }
  const int32_t pwm = (int32_t)EUP_PWM_NEUTRAL_US +
                      (int32_t)(value * (float)EUP_PWM_SPAN_US);
  return (uint16_t)pwm;
}

bool eup_parse_thruster_packet(const uint8_t *frame,
                               size_t length,
                               eup_thruster_packet_t *packet) {
  /* magic + version/sequence + 8 uint16 PWM channels + uint16 CRC */
  const size_t expected_length = 4u + 2u + (EUP_THRUSTER_COUNT * 2u) + 2u;
  if (frame == 0 || packet == 0 || length != expected_length) {
    return false;
  }
  if (frame[0] != EUP_PACKET_MAGIC_0 || frame[1] != EUP_PACKET_MAGIC_1 ||
      frame[2] != EUP_PACKET_MAGIC_2 || frame[3] != EUP_PACKET_MAGIC_3) {
    return false;
  }

  const uint16_t expected_crc = (uint16_t)frame[length - 2u] |
                                ((uint16_t)frame[length - 1u] << 8);
  /* Reject corrupt packets before exposing any PWM values to callers. */
  if (eup_crc16_ccitt(frame, length - 2u) != expected_crc) {
    return false;
  }

  packet->version = frame[4];
  packet->sequence = frame[5];
  for (uint8_t index = 0; index < EUP_THRUSTER_COUNT; ++index) {
    /* Payload PWM values are little-endian to match the ROS-side builder. */
    const size_t offset = 6u + (size_t)index * 2u;
    packet->pwm_us[index] = (uint16_t)frame[offset] |
                            ((uint16_t)frame[offset + 1u] << 8);
  }
  return packet->version == EUP_PACKET_VERSION;
}
