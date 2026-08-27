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

RS1_PKG_NAME = "rs1_sim"
CONFIG_PATH = "config"
WORLDS_PATH = "worlds"
IGN_SERVER_CONFIG = "ignition_server.config"

WORLD_ARG = "world"


def generate_launch_description():
    pkg_path = FindPackageShare(RS1_PKG_NAME)
    config_path = PathJoinSubstitution([pkg_path, CONFIG_PATH])

    world = LaunchConfiguration(WORLD_ARG)
    use_sim_time = LaunchConfiguration("use_sim_time")

    server_config = PathJoinSubstitution([config_path, IGN_SERVER_CONFIG])

    return LaunchDescription(
        [
            # Setup launch args
            # TODO (james): Clean up once world is created
            DeclareLaunchArgument(
                "world",
                default_value="v2_forest",
                choices=["large_demo", "simple_trees", "v1_forest", "v2_forest"],
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
            ),
            SetEnvironmentVariable(
                "IGN_GAZEBO_SERVER_CONFIG_PATH",
                server_config,
            ),
            SetEnvironmentVariable(
                "GZ_SIM_SERVER_CONFIG_PATH",
                server_config,
            ),
            IncludeLaunchDescription(
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
                    "gz_args": [
                        PathJoinSubstitution(
                            [
                                pkg_path,
                                WORLDS_PATH,
                                [world, ".sdf"],
                            ]
                        ),
                        " -r",
                    ],
                }.items(),
            ),
            # Bridge clock
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge_clock",
                output="screen",
                parameters=[
                    {
                        "config_file": PathJoinSubstitution(
                            [
                                config_path,
                                "gazebo_bridge_clock.yml",
                            ]
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
