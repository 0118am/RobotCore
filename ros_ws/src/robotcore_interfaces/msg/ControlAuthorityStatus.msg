std_msgs/Header header
string selected_source
bool armed
uint64 arm_generation
bool abort_active
bool fault_latched
string fault_code
string message
float64 selected_command_age_s
float64 body_state_age_s
float64 target_age_s
# Symmetric bound applied to each direct thruster action in [-1.0, 1.0].
float64 action_limit
float64 action_slew_rate
string localization_source
