cmake_minimum_required(VERSION 3.8)

project(sample_gen)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

install(DIRECTORY config launch models sample_gen DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(
    PROGRAMS scripts/data_preprocessor.py
             scripts/model_importer.py
             scripts/sample_collector.py
             scripts/sample_generator.py
             scripts/model_tf_broadcaster.py
             scripts/robot_world_broadcaster.py
             scripts/yolo_inference.py
             scripts/yolo_format.py
             scripts/camera_collector.py
             scripts/parrot_world_broadcaster.py
             scripts/parrot_patrol.py
             scripts/make_labeled_world.py
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    # This might become interesting in the future find_package(ament_lint_auto REQUIRED)
    # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
