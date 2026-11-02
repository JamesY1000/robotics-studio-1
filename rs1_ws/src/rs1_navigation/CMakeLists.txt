cmake_minimum_required(VERSION 3.10)
project(rs1_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rs1_interfaces REQUIRED)

# Include the 'include' directory
include_directories(include)

add_executable(terrain_following
  src/terrain_following.cpp
  src/terrain_following_node.cpp
)

add_executable(velocity_selector
  src/velocity_selector.cpp
  src/velocity_selector_node.cpp
)

# Declare version required by std::clamp =
target_compile_features(velocity_selector PRIVATE cxx_std_17)

ament_target_dependencies(terrain_following rclcpp action_msgs geometry_msgs sensor_msgs)
ament_target_dependencies(velocity_selector rclcpp geometry_msgs sensor_msgs rs1_interfaces)

# Install cpp executables
install(TARGETS
  terrain_following
  velocity_selector
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
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_controller_input test/test_controller_input.py
    TIMEOUT 90
    ENV VELOCITY_SELECTOR_EXECUTABLE=$<TARGET_FILE:velocity_selector>
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(test_velocity_selector test/test_velocity_selector.py
    TIMEOUT 60
    ENV VELOCITY_SELECTOR_EXECUTABLE=$<TARGET_FILE:velocity_selector>
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(test_terrain_following test/test_terrain_following.py
    TIMEOUT 90
    ENV TERRAIN_FOLLOWING_EXECUTABLE=$<TARGET_FILE:terrain_following>
  )
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
