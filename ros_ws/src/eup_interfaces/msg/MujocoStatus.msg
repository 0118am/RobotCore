# Runtime status for the MuJoCo simulation backend.
std_msgs/Header header
string model_path
bool model_loaded
bool paused
bool viewer_enabled
bool viewer_running
uint64 step_count
float64 sim_time_s
float32 publish_rate_hz
string last_command
string last_error
