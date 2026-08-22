# Normalized 8-thruster command shared by policy/control and edge hardware.
std_msgs/Header header
# Fixed BlueROV2 Heavy-style channel order, each value in [-1.0, 1.0].
float32[8] normalized
# False means downstream hardware should disable or neutralize outputs.
bool enable
# Persistent authority state is distinct from the momentary dead-man/output
# enable. A bridge or board restart requires a later arm generation.
bool armed
uint64 arm_generation
# Exact producer identity, for example pid_controller or command_authority:pid.
string source
