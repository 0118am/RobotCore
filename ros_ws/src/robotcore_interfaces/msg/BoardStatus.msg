# Low-level board status reported by the real Aboard bridge.
std_msgs/Header header
bool connected
bool heartbeat_ok
bool failsafe_active
# Protocol-v2 transport state. One control session and one monotonic sequence
# replace the previous unacknowledged 16-channel heartbeat stream.
bool session_established
bool outputs_enabled
# Saved gyro-bias calibration performed inside the external IMU through
# Aquaboard. Exactly one terminal flag is reported after completion.
bool imu_gyro_calibration_active
bool imu_gyro_calibration_succeeded
bool imu_gyro_calibration_failed
uint8 protocol_version
uint32 board_tick_ms
uint32 boot_id
uint8 reset_cause
uint32 control_session_id
uint32 received_sequence
uint32 applied_sequence
uint16 command_age_ms
uint16 rx_crc_errors
uint8 safety_reason
# PWM compare values confirmed by the MCU at its update boundary for physical
# outputs PWM 8-15. This is applied-command echo, not ESC or motor feedback.
uint16[8] pwm_us
