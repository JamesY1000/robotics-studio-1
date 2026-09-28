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
