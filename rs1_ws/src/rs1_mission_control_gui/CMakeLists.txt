cmake_minimum_required(VERSION 3.8)
project(rs1_mission_control_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rs1_mission_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(include)

# Plain executable - no pluginlib, no plugin.xml. Headers with Q_OBJECT
# are listed explicitly so AUTOMOC mocs them.
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/main_window.cpp
  src/map_widget.cpp
  src/robot_card_widget.cpp
  include/rs1_mission_control_gui/main_window.hpp
  include/rs1_mission_control_gui/map_widget.hpp
  include/rs1_mission_control_gui/robot_card_widget.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  rs1_mission_interfaces
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
