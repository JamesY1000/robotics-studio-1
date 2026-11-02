std_msgs/Header header
string session_id
# Unique for this coordinator process, including manually launched sessions.
string instance_id
# Configured target in its source frame, before conversion for the executors.
geometry_msgs/PoseStamped roi
string state
string parrot_stage
string husky_stage
string message
bool cleanup_complete
# auto (Autonomous), husky1 (husky teleop), parrot1 (parrot teleop), or hold (stop)
string control_mode
# Only one manual input can command the selected robot
string manual_input
# Changes when input or control is reselected so quick switches also clear held input
uint64 input_revision
