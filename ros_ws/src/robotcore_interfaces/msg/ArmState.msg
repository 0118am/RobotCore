# Generic manipulator state for simulated and real arm backends.
std_msgs/Header header
# Joint arrays share this ordering.
string[] joint_names
float64[] position
float64[] velocity
float64[] effort
# False lets policies report missing/invalid input without crashing.
bool state_valid
