# Low-level board status reported by the real Aboard bridge.
std_msgs/Header header
bool connected
bool heartbeat_ok
bool estop_active
bool failsafe_active
# Last applied or reported PWM values for all low-level motor outputs.
uint16[] pwm_us
float32 bus_voltage_v
float32 board_temp_c
string firmware_version
