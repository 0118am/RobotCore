# Target command consumed by trajectory-tracking body policies. Its pose and
# vectors use the right-handed frame named by header.frame_id (RobotCore: FLU).
std_msgs/Header header
geometry_msgs/Pose target_pose
geometry_msgs/Twist target_twist
# Target acceleration is expressed in the same frame as header.frame_id.
geometry_msgs/Accel target_accel
string trajectory_type
# Controller profile that consumes this target. Trajectory generation and
# execution mode are intentionally independent (for example, a spatial
# figure-eight can be executed by station_hold or station_hold_fast).
string control_mode
# Explicit generator phase. Controllers use this only to remove intermediate
# translation setpoint caps during the finite center-approach segment; actuator
# and attitude safety limits remain authoritative.
string trajectory_phase
float64 time_s
bool valid
