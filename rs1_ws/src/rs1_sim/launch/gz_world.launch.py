from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
Handles launching of gazebo server, world, and assets
"""

PKG_NAME = "rs1_sim"
CONFIG_PATH = "config"
WORLDS_PATH = "worlds"
IGN_SERVER_CONFIG = "ignition_server.config"

WORLD_ARG = "world"


def generate_launch_description():
    package_share = FindPackageShare(PKG_NAME)
    config_directory = PathJoinSubstitution([package_share, "config"])

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    world_file = PathJoinSubstitution(
        [
            package_share,
            "worlds",
            [world, ".sdf"],
        ]
    )

    server_config = PathJoinSubstitution(
        [
            config_directory,
            "ignition_server.config",
        ]
    )

    clock_bridge_config = PathJoinSubstitution(
        [
            config_directory,
            "gazebo_bridge_clock.yml",
        ]
    )

    # Launch arguments
    world_argument = DeclareLaunchArgument(
        "world",
        default_value="v2_forest",
        choices=["large_demo", "simple_trees", "v1_forest", "v2_forest"],
        description="Gazebo world to load.",
    )

    use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use the Gazebo simulation clock.",
    )

    # Set gazebo server config
    set_ignition_server_config = SetEnvironmentVariable(
        "IGN_GAZEBO_SERVER_CONFIG_PATH",
        server_config,
    )

    set_gazebo_server_config = SetEnvironmentVariable(
        "GZ_SIM_SERVER_CONFIG_PATH",
        server_config,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": [world_file, " -r"],
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_clock",
        output="screen",
        parameters=[
            {
                "config_file": clock_bridge_config,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            world_argument,
            use_sim_time_argument,
            set_ignition_server_config,
            set_gazebo_server_config,
            gazebo,
            clock_bridge,
        ]
    )
