std_msgs/Header header

uint8 classification

# Offset between pose of model and center of bounding box (for Gazebo)
float32 offset_x
float32 offset_y
float32 offset_z

# 3D Pose of bounding box
geometry_msgs/Pose pose

# Size of bounding box
geometry_msgs/Vector3 size
