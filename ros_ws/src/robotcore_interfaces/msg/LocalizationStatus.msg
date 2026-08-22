# Quantitative localisation health accompanying /robot/body_state.
std_msgs/Header header

# Source timestamps. A zero stamp means that source has not produced a usable
# sample since startup.
builtin_interfaces/Time last_vio_stamp
builtin_interfaces/Time last_tag_stamp
builtin_interfaces/Time last_absolute_fix_stamp

# Age is measured from the source Header stamp, not merely callback arrival.
# Unavailable values are positive infinity.
float32 vio_age_s
float32 tag_age_s
float32 absolute_fix_age_s
float32 apriltag_frame_age_s

# Rates are measured from callback arrivals over a rolling window.
float32 vio_rate_hz
float32 tag_rate_hz
float32 body_state_rate_hz
float32 apriltag_frame_rate_hz

# now - Header.stamp measured when the sample arrived. These include sensor
# processing and transport latency. Unavailable values are NaN.
float32 vio_transport_delay_s
float32 tag_transport_delay_s

# Time-aligned Tag-versus-current-map-alignment innovation. Unavailable values are NaN.
float32 tag_vio_translation_residual_m
float32 tag_vio_angle_residual_deg

# Covariance corresponding to the canonical /robot/body_state EKF estimate.
float64[36] pose_covariance
float64[36] twist_covariance

int32 detected_tag_count
int32 mapped_tag_count
int32 inlier_tag_count
float32 tag_reprojection_rms_px
float32 minimum_tag_edge_px
bool tag_pose_published
bool vio_fresh
bool tag_fresh
bool tag_consistent
bool absolute_fix_valid
bool position_estimated
string localization_source
# "primary", "degraded", or empty when no fresh Tag pose is available.
string tag_observation_class
string apriltag_rejection_reason
string rejection_reason
