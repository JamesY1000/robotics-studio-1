import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
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
        default_value="robot_t",
        choices=["robot_m", "robot_s", "robot_experimental", "robot_t"],
        description="Robot Model",
    ),
    DeclareLaunchArgument("backpack", default_value="none", description="Name of the robot backpack"),
]


def generate_launch_description():
    robot_sim_pkg_dir = get_package_share_directory("robot_sim")
    sample_gen_pkg_dir = get_package_share_directory("sample_gen")
    simulation_launch_file = PathJoinSubstitution([robot_sim_pkg_dir, "launch", "sim.launch.py"])
    config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "config.yml"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simulation_launch_file]),
        launch_arguments=[
            ("world", LaunchConfiguration("world")),
            ("rviz", LaunchConfiguration("rviz")),
            ("model", LaunchConfiguration("model")),
        ],
    )

    model_importer = Node(
        package="sample_gen",
        name="model_importer_node",
        executable="model_importer.py",
        parameters=[config, {"use_sim_time": True}],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        on_exit=launch.actions.Shutdown(),
        output="screen",
    )

    sample_collector_node = Node(
        package="sample_gen",
        name="sample_collector_node",
        executable="sample_collector.py",
        parameters=[config, {"use_sim_time": True}],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        on_exit=launch.actions.Shutdown(),
        output="screen",
    )

    model_tf_broadcaster = Node(
        package="sample_gen",
        name="model_tf_broadcaster_node",
        executable="model_tf_broadcaster.py",
        on_exit=launch.actions.Shutdown(),
        output="screen",
    )

    robot_world_broadcaster = Node(
        package="sample_gen",
        name="robot_world_broadcaster_node",
        executable="robot_world_broadcaster.py",
        on_exit=launch.actions.Shutdown(),
        output="screen",
    )

    ld = LaunchDescription(ARGUMENTS)

    # TODO (james): Handle elegant shutdown of gazebo on model_importer termination
    ld.add_action(gazebo)
    ld.add_action(model_tf_broadcaster)
    ld.add_action(robot_world_broadcaster)
    ld.add_action(sample_collector_node)
    ld.add_action(model_importer)

    return ld


if __name__ == "__main__":
    generate_launch_description()
