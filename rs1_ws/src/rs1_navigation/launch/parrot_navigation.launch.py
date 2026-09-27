from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

"""
Launches Nav2, terrain following, and guided exploration for Parrot.
"""

PKG_NAME = "rs1_navigation"
NAV2_PKG_NAME = "nav2_bringup"
FRONTIER_EXPLORATION_NAME = "explore_lite"
ROBOT_NAMESPACE = "parrot1"
ROBOT = "parrot"
NAV2_CONFIG = "nav2_params_parrot1.yml"
TERRAIN_FOLLOWING_CONFIG = "terrain_following_params.yml"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    nav2_log_level = LaunchConfiguration("nav2_log_level")
    package_share = FindPackageShare(PKG_NAME)
    nav2_package_share = FindPackageShare(NAV2_PKG_NAME)
    exploration_package_share = FindPackageShare(FRONTIER_EXPLORATION_NAME)

    nav2_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            ROBOT,
            NAV2_CONFIG,
        ]
    )

    terrain_following_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            TERRAIN_FOLLOWING_CONFIG,
        ]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use gz sim clock.",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=nav2_config,
        description="Nav2 parameter file.",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically activate the Nav2 lifecycle nodes.",
    )

    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Respawn Nav2 nodes if they crash.",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Exploration and terrain-following logging level.",
    )

    nav2_log_level_arg = DeclareLaunchArgument(
        "nav2_log_level",
        default_value="warn",
        description="Nav2 logging level.",
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    nav2_package_share,
                    "launch",
                    "navigation_launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": ROBOT_NAMESPACE,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": "False",
            "use_respawn": use_respawn,
            "log_level": nav2_log_level,
        }.items(),
    )

    terrain_following_node = Node(
        package=PKG_NAME,
        executable="terrain_following",
        name="terrain_following_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            terrain_following_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    guided_frontier_exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [exploration_package_share, "launch", "explore.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": "",
            "robot_base_frame": f"{ROBOT_NAMESPACE}_base_link",
            "use_sim_time": use_sim_time,
            "log_level": log_level,
        }.items(),
    )

    navigation = GroupAction(
        scoped=True,
        actions=[
            PushRosNamespace(ROBOT_NAMESPACE),  # Remap to namespace
            SetRemap(src="/map", dst="map"),
            SetRemap(src="/map_updates", dst="map_updates"),
            GroupAction(scoped=True, actions=[nav2_launch]),
            terrain_following_node,
            guided_frontier_exploration_launch,
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            autostart_arg,
            use_respawn_arg,
            log_level_arg,
            nav2_log_level_arg,
            navigation,
        ]
    )
