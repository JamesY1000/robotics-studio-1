cmake_minimum_required(VERSION 3.10)
project(rs1_dashboard)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rs1_interfaces REQUIRED)

add_executable(emergency_stop
  src/emergency_stop.cpp
  src/emergency_stop_node.cpp
)
target_include_directories(emergency_stop PRIVATE include)
ament_target_dependencies(emergency_stop rclcpp geometry_msgs std_msgs rs1_interfaces)

install(TARGETS emergency_stop DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/telemetry_viewer.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(telemetry_launch_test test/test_telemetry_launch.py)
endif()

ament_package()
