#include "eup_firmware/packet.h"

static void apply_pwm_outputs(const uint16_t pwm_us[EUP_THRUSTER_COUNT]) {
  /*
   * Board-specific PWM writes belong here after Phase 0 confirms the Aboard
   * variant, timer channels, PWM frequency, and output pins.
   */
  (void)pwm_us;
}

static void apply_failsafe_outputs(void) {
  /* Neutral PWM is the safe fallback for boot, heartbeat timeout, and estop. */
  uint16_t neutral[EUP_THRUSTER_COUNT] = {
      EUP_PWM_NEUTRAL_US, EUP_PWM_NEUTRAL_US, EUP_PWM_NEUTRAL_US,
      EUP_PWM_NEUTRAL_US, EUP_PWM_NEUTRAL_US, EUP_PWM_NEUTRAL_US,
      EUP_PWM_NEUTRAL_US, EUP_PWM_NEUTRAL_US};
  apply_pwm_outputs(neutral);
}

int main(void) {
  eup_thruster_packet_t packet;
  (void)packet;

  /* Start safe before interrupts, transports, or scheduler hooks are added. */
  apply_failsafe_outputs();

  while (1) {
    /*
     * Board-specific loop placeholder:
     * - read serial or CAN frame
     * - parse with eup_parse_thruster_packet
     * - refresh heartbeat
     * - apply PWM
     * - publish board status
     */
  }
}
