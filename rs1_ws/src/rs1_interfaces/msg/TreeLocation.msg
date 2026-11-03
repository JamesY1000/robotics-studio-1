string tree_type
geometry_msgs/Point location
# How many detections were merged into this tree.
uint32 observations
# Detection confidence (0..1) of the observation that produced this location.
float32 score
# Lidar-measured trunk/cone width at the detection bearing [m] (0.0 = unknown).
float32 trunk_width_m