# Command modes kept in one message so policy experiments do not change topics.
uint8 COMMAND_JOINT_TARGET=0
uint8 COMMAND_JOINT_DELTA=1
uint8 COMMAND_EE_DELTA=2

std_msgs/Header header
uint8 command_type
# Names define the ordering for joint_targets and joint_deltas.
string[] joint_names
float64[] joint_targets
float64[] joint_deltas
# Used when command_type is COMMAND_EE_DELTA.
geometry_msgs/Twist ee_delta
bool enable
string source
