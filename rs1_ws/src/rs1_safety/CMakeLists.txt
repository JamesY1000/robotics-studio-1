cmake_minimum_required(VERSION 3.10)
project(rs1_safety)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rs1_interfaces REQUIRED)

add_executable(emergency_stop
  src/emergency_stop.cpp
  src/emergency_stop_node.cpp
)
target_include_directories(emergency_stop PRIVATE include)
ament_target_dependencies(emergency_stop rclcpp geometry_msgs rs1_interfaces)

install(TARGETS emergency_stop DESTINATION lib/${PROJECT_NAME})

ament_package()
