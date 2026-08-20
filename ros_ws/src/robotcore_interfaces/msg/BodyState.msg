# Body state consumed by body policy, UI, logging, and control tools.
std_msgs/Header header
# Pose is expressed in header.frame_id (normally map, using the measured RobotCore
# global FLU convention: +X forward, +Y left, +Z up). Twist is expressed in
# base_link using the same right-handed FLU axis directions.
geometry_msgs/Pose pose
geometry_msgs/Twist twist
# True only when all three linear twist components originate from a velocity
# estimator or measurement.  A default-initialised Vector3 is not a zero-speed
# observation.
bool linear_velocity_valid
# True when pose is a ZED-VIO local-motion prediction following the last
# trusted map observation. In that case state_valid remains false so control
# code cannot mistake the prediction for an absolute localisation fix.
bool position_estimated
# Human-readable provenance for the current map pose, for example
# "AprilTag+ZED VIO" or "ZED VIO after Tag loss". An empty value means no
# usable position estimate is available.
string localization_source
bool state_valid
