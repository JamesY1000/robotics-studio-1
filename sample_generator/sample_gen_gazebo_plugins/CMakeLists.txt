cmake_minimum_required(VERSION 3.5)
project(sample_gen_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sample_gen_msgs REQUIRED)

add_library(object_label_plugin SHARED src/object_label_plugin.cpp)

include_directories(include)
ament_target_dependencies(
    object_label_plugin
    rclcpp
    tf2_msgs
    geometry_msgs
    sample_gen_msgs)

target_link_libraries(object_label_plugin ignition-gazebo6::ignition-gazebo6)

install(TARGETS object_label_plugin DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
