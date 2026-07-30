# Target command consumed by trajectory-tracking body policies. Its pose and
# vectors use the right-handed frame named by header.frame_id (EUP: FLU).
std_msgs/Header header
geometry_msgs/Pose target_pose
geometry_msgs/Twist target_twist
# Target acceleration is expressed in the same frame as header.frame_id.
geometry_msgs/Accel target_accel
string trajectory_type
float64 time_s
bool valid
