import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Start rviz.",
    ),
    DeclareLaunchArgument(
        "world",
        default_value="empty",
        description="World to launch in Gazebo.",
    ),
    DeclareLaunchArgument(
        "model",
        default_value="arculee_t",
        description="Model to launch in Gazebo.",
    ),
]


def generate_launch_description():
    arculee_sim_pkg_dir = get_package_share_directory("arculee_sim")
    sample_gen_pkg_dir = get_package_share_directory("sample_gen")
    laser_converter_pkg_dir = get_package_share_directory("laser_converter")

    laser_converter_launch_file = PathJoinSubstitution([laser_converter_pkg_dir, "launch", "laser_converter.launch.py"])

    simulation_launch_file = PathJoinSubstitution([arculee_sim_pkg_dir, "launch", "sim.launch.py"])
    rviz_config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "sample_gen.rviz"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simulation_launch_file]),
        launch_arguments=[
            ("world", LaunchConfiguration("world")),
            ("rviz", "false"),
            ("model", LaunchConfiguration("model")),
        ],
    )

    laser_converter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([laser_converter_launch_file]),
        launch_arguments=[("use_sim_time", "true"), ("log_level", "info"), ("namespace", "")],
    )

    rviz = Node(
        package="rviz2",
        name="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    model_tf_broadcaster = Node(
        package="sample_gen",
        name="model_tf_broadcaster_node",
        executable="model_tf_broadcaster.py",
        on_exit=launch.actions.Shutdown(),
        output="screen",
    )

    arculee_world_broadcaster = Node(
        package="sample_gen",
        name="arculee_world_broadcaster_node",
        executable="arculee_world_broadcaster.py",
        on_exit=launch.actions.Shutdown(),
        output="screen",
    )

    # yolo_inference_node = Node(
    #     package="sample_gen",
    #     name="yolo_inference_node",
    #     executable="yolo_inference.py",
    #     on_exit=launch.actions.Shutdown(),
    #     output="screen",
    # )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(rviz)
    ld.add_action(gazebo)
    ld.add_action(model_tf_broadcaster)
    ld.add_action(arculee_world_broadcaster)
    ld.add_action(laser_converter_node)
    # ld.add_action(yolo_inference_node) # Run as separate node - interactive terminal required

    return ld


if __name__ == "__main__":
    generate_launch_description()
