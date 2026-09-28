cmake_minimum_required(VERSION 3.10)
project(rs1_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rs1_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Include the 'include' directory
include_directories(include)

# Create executables
add_executable(mission_coordinator
  src/mission_coordinator.cpp
  src/mission_coordinator_node.cpp
)
add_executable(husky_mission_executor
  src/husky_mission_executor.cpp
  src/husky_mission_executor_node.cpp
)
add_executable(parrot_mission_executor
  src/parrot_mission_executor.cpp
  src/parrot_mission_executor_node.cpp
)

ament_target_dependencies(mission_coordinator tf2 tf2_ros tf2_geometry_msgs rclcpp rcl_interfaces rclcpp_action std_msgs geometry_msgs visualization_msgs rs1_interfaces std_srvs)
ament_target_dependencies(husky_mission_executor rclcpp rclcpp_action std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs nav2_msgs rs1_interfaces)
ament_target_dependencies(parrot_mission_executor rclcpp rclcpp_action std_msgs geometry_msgs tf2 tf2_ros nav2_msgs rs1_interfaces)

# Install cpp executables
install(TARGETS
  mission_coordinator
  husky_mission_executor
  parrot_mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
