std_msgs/Header header
string session_id
# Unique for this coordinator process, including manually launched sessions.
string instance_id
# Configured target in its source frame, before conversion for the executors.
# roi is the entry point (first-placed ROI corner) used for traversal.
geometry_msgs/PoseStamped roi
# Axis-aligned ROI box minimo/maximum in the ROI frame; unused while no
# sampling phase is configured (min == max).
geometry_msgs/Point roi_min
geometry_msgs/Point roi_max
# Fraction of the sampling sweep completed, 0.0 - 1.0.
float32 roi_progress
string state
string parrot_stage
string husky_stage
string message
bool cleanup_complete
# Stays true while stopping the survey, returning, or assisting the home journey
bool returning_home
# auto (Autonomous), husky1 (husky teleop), parrot1 (parrot teleop), or hold (stop)
string control_mode
# Only one manual input can command the selected robot
string manual_input
# Changes when input or control is reselected so quick switches also clear held input
uint64 input_revision
# Safety state processed by the coordinator, used to acknowledge a stop before reset
string safety_instance_id
uint64 safety_revision
