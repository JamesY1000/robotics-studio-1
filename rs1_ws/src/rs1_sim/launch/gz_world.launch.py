import shlex
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
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
    log_level = LaunchConfiguration("log_level")

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
        choices=["large_demo", "simple_trees", "v1_forest", "v2_forest", "v2_forest_labeled"],
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

    def launch_gazebo(context):
        override = LaunchConfiguration("world_file").perform(context)
        selected = override or world_file.perform(context)
        headless = LaunchConfiguration("headless").perform(context).lower() == "true"
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments={
                # -s runs the server only; --headless-rendering is REQUIRED for
                # the server to render sensor data (gpu_lidar point clouds, rgbd
                # camera) without a GUI. Without it the render loop never starts
                # and sensor topics go stale.
                "gz_args": f"{shlex.quote(selected)} -r" + (" -s --headless-rendering" if headless else ""),
                # End the world marker/clock bridge when Gazebo exits or fails.
                "on_exit_shutdown": "true",
            }.items())]

    gazebo = OpaqueFunction(function=launch_gazebo)

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_clock",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
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
            DeclareLaunchArgument("world_file", default_value=""),
            DeclareLaunchArgument("headless", default_value="False"),
            use_sim_time_argument,
            DeclareLaunchArgument("log_level", default_value="warn",
                                  description="Clock bridge logging level."),
            set_ignition_server_config,
            set_gazebo_server_config,
            gazebo,
            clock_bridge,
            # The world owns the stop authority so closing the dashboard cannot remove it
            Node(package="rs1_safety", executable="emergency_stop", output="screen"),
        ]
    )
