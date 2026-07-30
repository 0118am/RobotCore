# Tracking diagnostics published by runtime from ROS trajectory/body topics.
std_msgs/Header header
string trajectory_type
bool valid
float64 time_s

# Positions are expressed in header.frame_id, normally map. EUP uses the
# right-handed FLU convention: +X forward, +Y left, +Z up.
geometry_msgs/Point target_position
geometry_msgs/Point actual_position

# Velocity and acceleration fields are expressed in base_link/body frame so they
# can be compared directly with BodyState.twist.
geometry_msgs/Vector3 target_velocity_body
geometry_msgs/Vector3 actual_velocity_body
geometry_msgs/Vector3 target_acceleration_body
geometry_msgs/Vector3 actual_acceleration_body

float64 position_error_m
float64 velocity_error_mps
float64 speed_mps
float64 acceleration_mps2
