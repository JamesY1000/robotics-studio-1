cmake_minimum_required(VERSION 3.10)
project(example_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Include the 'include' directory
include_directories(include)

# Create executables
add_executable(example_node_cpp src/example_node_cpp.cpp)
# ament_target_dependencies(example_node_cpp rclcpp std_msgs)
target_link_libraries(example_node_cpp
  PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

# Install cpp executables
install(TARGETS
  example_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/example_node_py.py
  DESTINATION lib/${PROJECT_NAME}
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
