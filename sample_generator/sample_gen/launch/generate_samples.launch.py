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
        description="World to launch simulation with",
    ),
    DeclareLaunchArgument(
        "model",
        default_value="arculee_t",
        choices=["arculee_m", "arculee_s", "arculee_experimental", "arculee_t"],
        description="Arculee Model",
    ),
    DeclareLaunchArgument("backpack", default_value="none", description="Name of the arculee backpack"),
]


def generate_launch_description():
    arculee_sim_pkg_dir = get_package_share_directory("arculee_sim")
    sample_gen_pkg_dir = get_package_share_directory("sample_gen")
    laser_converter_pkg_dir = get_package_share_directory("laser_converter")

    laser_converter_launch_file = PathJoinSubstitution([laser_converter_pkg_dir, "launch", "laser_converter.launch.py"])

    simulation_launch_file = PathJoinSubstitution([arculee_sim_pkg_dir, "launch", "sim.launch.py"])
    rviz_config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "sample_gen.rviz"])

    config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "config.yml"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simulation_launch_file]),
        launch_arguments=[
            ("world", LaunchConfiguration("world")),
            ("model", LaunchConfiguration("model")),
            ("rviz", "false"),
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

    sample_collector_node = Node(
        package="sample_gen",
        name="sample_collector_node",
        executable="sample_collector.py",
        parameters=[config, {"use_sim_time": True}],
        on_exit=launch.actions.Shutdown(),
        # arguments=["--ros-args", "--log-level", "debug"],
        output="screen",
    )

    sample_generator_node = Node(
        package="sample_gen",
        name="sample_generator_node",
        executable="sample_generator.py",
        parameters=[config, {"use_sim_time": True}],
        on_exit=launch.actions.Shutdown(),
        # arguments=["--ros-args", "--log-level", "debug"],
        output="screen",
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

    ld = LaunchDescription(ARGUMENTS)

    # TODO (james): Handle elegant shutdown of gazebo on model_importer termination

    ld.add_action(rviz)
    ld.add_action(gazebo)
    ld.add_action(model_tf_broadcaster)
    ld.add_action(arculee_world_broadcaster)
    ld.add_action(laser_converter_node)
    ld.add_action(sample_collector_node)
    ld.add_action(sample_generator_node)

    return ld


if __name__ == "__main__":
    generate_launch_description()
