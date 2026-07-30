# Runtime status for BodyPolicy and ArmPolicy nodes.
std_msgs/Header header
# Expected values: "body" or "arm".
string policy_role
string policy_name
# Runner type from policy.yaml, such as dummy, onnx, pth, or mmn.
string runner
string model_path
bool loaded
bool input_ready
# Names of missing or stale required inputs.
string[] missing_inputs
float32 inference_latency_ms
uint32 inference_count
