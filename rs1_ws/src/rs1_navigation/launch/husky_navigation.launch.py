from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

"""
Handles launching Nav2 for Husky.
"""

PKG_NAME = "rs1_navigation"
NAV2_PKG_NAME = "nav2_bringup"
ROBOT_NAMESPACE = "husky1"
ROBOT = "husky"
NAV2_CONFIG = "nav2_params_husky1.yml"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    package_share = FindPackageShare(PKG_NAME)
    nav2_package_share = FindPackageShare(NAV2_PKG_NAME)

    nav2_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            ROBOT,
            NAV2_CONFIG,
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
        description="Nav2 params.",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically activate Nav2 lifecycle nodes.",
    )

    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Respawn Nav2 nodes if they crash.",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
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
            "log_level": log_level,
        }.items(),
    )

    navigation = GroupAction(
        scoped=True,
        actions=[
            PushRosNamespace(ROBOT_NAMESPACE),  # Remap to namespace
            SetRemap(src="/map", dst="map"),
            SetRemap(src="/map_updates", dst="map_updates"),
            nav2_launch,
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            autostart_arg,
            use_respawn_arg,
            log_level_arg,
            navigation,
        ]
    )
