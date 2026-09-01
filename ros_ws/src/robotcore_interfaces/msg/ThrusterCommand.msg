# Direct 8-thruster action shared by policy/control and edge hardware.
std_msgs/Header header
# Fixed T1..T8 channel order. Each action is in [-1.0, 1.0]. PID and manual
# actions map as PWM_us = 1500 + 250 * action. The hardware bridge applies the
# deployed RL policy's T5/T6 sign adapter only to source command_authority:rl.
float32[8] action
# False means downstream hardware should disable or neutralize outputs.
bool enable
# Persistent authority state is distinct from the momentary dead-man/output
# enable. A bridge or board restart requires a later arm generation.
bool armed
uint64 arm_generation
# Exact producer identity, for example pid_controller or command_authority:pid.
string source
