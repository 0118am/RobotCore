# Single mapped AprilTag observation contract consumed by the ESKF.
# Invalid observations still carry quality/rejection metadata. A monotonically
# increasing map_generation folds relocalization into this same input stream.
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
uint64 map_generation
bool relocalization_requested
bool pose_valid
int32 detected_tag_count
int32 mapped_tag_count
int32 inlier_tag_count
float32 reprojection_rms_px
float32 minimum_tag_edge_px
string rejection_reason
