# Per-camera-frame quality result from mapped AprilTag localisation.
std_msgs/Header header
int32 detected_tag_count
int32 mapped_tag_count
int32 inlier_tag_count
float32 reprojection_rms_px
float32 minimum_tag_edge_px
bool pose_published
bool degraded
string rejection_reason
