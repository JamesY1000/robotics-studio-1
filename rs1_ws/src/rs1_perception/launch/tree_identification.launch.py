from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rs1_perception")

    yolo_config = PathJoinSubstitution([package_share, "config", "yolo.yaml"])
    tree_location_config = PathJoinSubstitution(
        [package_share, "config", "tree_location.yaml"]
    )
    model_path = PathJoinSubstitution([package_share, "models", "best.pt"])

    # Publishes world -> parrot1_base_link (+ the parrot static chain on /tf_static)
    # so the tree-location node can transform camera points into the global frame.
    world_broadcaster = Node(
        package="rs1_perception",
        executable="parrot_world_broadcaster.py",
        name="parrot_world_broadcaster",
        output="screen",
    )

    # YOLO tree detection, exposed through the /yolo/detect service.
    yolo_node = Node(
        package="rs1_perception",
        executable="yolo_node.py",
        name="yolo_node",
        output="screen",
        parameters=[yolo_config, {"model_path": model_path}],
    )

    # Depth + lidar -> world-frame tree base locations, exposed through the
    # /yolo/process_tree_locations service.
    tree_location_node = Node(
        package="rs1_perception",
        executable="tree_location_processing.py",
        name="tree_location_node",
        output="screen",
        parameters=[tree_location_config],
    )

    return LaunchDescription(
        [
            world_broadcaster,
            yolo_node,
            tree_location_node,
        ]
    )