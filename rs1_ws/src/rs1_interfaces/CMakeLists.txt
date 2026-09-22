cmake_minimum_required(VERSION 3.10)
project(rs1_interfaces)

# TODO (james): Clean up dependencies

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        "msg/BoundingBox.msg"
        "msg/TreeLocation.msg"
        "msg/YoloDetections.msg"
        "srv/DetectYolo.srv"
        "srv/ProcessTreeLocations.srv"
        DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
