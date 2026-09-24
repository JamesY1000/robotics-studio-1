cmake_minimum_required(VERSION 3.5)
project(sample_gen_msgs)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    # Add msg files
    msg/ObjectLabel.msg
    msg/LoadedModelInfo.msg
    # Add srv files
    srv/CollectSample.srv
    srv/CheckModelsLoaded.srv
    srv/GetCurrentSampleID.srv
    srv/GetDBData.srv
    DEPENDENCIES
    std_msgs
    geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
