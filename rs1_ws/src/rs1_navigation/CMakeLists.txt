cmake_minimum_required(VERSION 3.10)
project(rs1_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include the 'include' directory
include_directories(include)

add_executable(terrain_following
  src/terrain_following.cpp
  src/terrain_following_node.cpp
)

ament_target_dependencies(terrain_following rclcpp geometry_msgs sensor_msgs)

# Install cpp executables
install(TARGETS
  terrain_following
  DESTINATION lib/${PROJECT_NAME}
)

# Install cpp executables
install(DIRECTORY
  config
  launch
  maps
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
