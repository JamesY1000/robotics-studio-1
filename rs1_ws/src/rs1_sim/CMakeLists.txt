cmake_minimum_required(VERSION 3.10)
project(rs1_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/simulation_session.py DESTINATION lib/${PROJECT_NAME})

# Install additional folders
install(
  DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(simulation_session_test test/test_simulation_session.py)
  ament_add_pytest_test(scoped_robot_launch_test test/test_scoped_robot_launch.py)
  ament_add_pytest_test(spawn_readiness_test test/test_spawn_readiness.py TIMEOUT 45)
  ament_add_pytest_test(rviz_launch_test test/test_rviz_launch.py)
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
