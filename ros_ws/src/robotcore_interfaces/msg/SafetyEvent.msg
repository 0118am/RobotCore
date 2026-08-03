# Safety event stream for logging and operator UI.
uint8 LEVEL_INFO=0
uint8 LEVEL_WARN=1
uint8 LEVEL_ERROR=2
uint8 LEVEL_ABORT=3

std_msgs/Header header
uint8 level
# Stable machine-readable code, for example ABORT_ACTIVE.
string code
string message
bool abort_active
string source
