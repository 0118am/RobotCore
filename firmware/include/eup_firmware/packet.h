#ifndef EUP_FIRMWARE_PACKET_H
#define EUP_FIRMWARE_PACKET_H

/*
 * Board-neutral packet contract for the low-level thruster controller.
 *
 * Frame layout:
 *   magic[4] = "EUP1"
 *   version[1]
 *   sequence[1]
 *   pwm_us[8] little-endian uint16
 *   crc16[2] little-endian CRC-16/CCITT over all preceding bytes
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EUP_PACKET_MAGIC_0 'E'
#define EUP_PACKET_MAGIC_1 'U'
#define EUP_PACKET_MAGIC_2 'P'
#define EUP_PACKET_MAGIC_3 '1'
#define EUP_PACKET_VERSION 1u
#define EUP_THRUSTER_COUNT 8u
#define EUP_PWM_NEUTRAL_US 1500u
#define EUP_PWM_SPAN_US 400u

typedef struct {
  uint8_t version;
  uint8_t sequence;
  /* PWM values are stored after normalized commands are mapped on Jetson. */
  uint16_t pwm_us[EUP_THRUSTER_COUNT];
} eup_thruster_packet_t;

uint16_t eup_crc16_ccitt(const uint8_t *data, size_t length);
/* Clamp normalized [-1, 1] input into the placeholder PWM window. */
uint16_t eup_normalized_to_pwm(float value);
bool eup_parse_thruster_packet(const uint8_t *frame,
                               size_t length,
                               eup_thruster_packet_t *packet);

#endif
