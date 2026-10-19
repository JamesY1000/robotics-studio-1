cmake_minimum_required(VERSION 3.10)
project(rs1_dashboard)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/telemetry_viewer.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(telemetry_launch_test test/test_telemetry_launch.py)
endif()

ament_package()
