# Normalized 8-thruster command shared by policy/control, MuJoCo, and edge hardware.
std_msgs/Header header
# Fixed BlueROV2 Heavy-style channel order, each value in [-1.0, 1.0].
float32[8] normalized
# False means downstream hardware should disable or neutralize outputs.
bool enable
# Human-readable producer such as thruster_allocator or safety_monitor.
string source
