# Feedback for the fixed 8-thruster command surface.
std_msgs/Header header
# Normalized actuator feedback in the same order as ThrusterCommand.normalized.
float32[8] normalized_feedback
# PWM-equivalent feedback from edge hardware.
uint16[] pwm_us
# Per-channel health flags from simulation or the low-level board.
bool[] healthy
