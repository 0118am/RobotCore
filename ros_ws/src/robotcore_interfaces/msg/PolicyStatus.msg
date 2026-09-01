# Runtime status for the body policy node.
std_msgs/Header header
# Expected value: "body".
string policy_role
string policy_name
# Runner type from policy.yaml, such as dummy, onnx, or tensorrt.
string runner
string model_path
bool loaded
bool input_ready
# Names of missing or stale required inputs.
string[] missing_inputs
float32 inference_latency_ms
uint32 inference_count
# Actual interval between policy timer callbacks. This includes scheduler delay,
# unlike inference_latency_ms, which measures TensorRT execution only.
float32 tick_interval_ms
# Number of intervals longer than 1.5 control periods since process start.
uint32 deadline_miss_count
