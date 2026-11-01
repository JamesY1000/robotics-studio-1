from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
Handles launching localisation and mapping nodes for parrot
"""

PKG_NAME = "rs1_localisation"
ROBOT_NAMESPACE = "parrot1"
ROBOT = "parrot"
EKF_CONFIG = "robot_localization_parrot1.yml"
SLAM_CONFIG = "slam_params_parrot1.yml"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_enabled = LaunchConfiguration("slam")
    log_level = LaunchConfiguration("log_level")
    package_share = FindPackageShare(PKG_NAME)

    ekf_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            ROBOT,
            EKF_CONFIG,
        ]
    )

    slam_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            ROBOT,
            SLAM_CONFIG,
        ]
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use gz sim clock.",
    )

    slam_arg = DeclareLaunchArgument(
        "slam", default_value="False", description="Start SLAM toolbox"
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=ROBOT_NAMESPACE,
        name="robot_localization",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            ekf_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("odometry/filtered", "odom"),
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace=ROBOT_NAMESPACE,
        name="slam_toolbox",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(slam_enabled),
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/map", "map"),
            ("/map_updates", "map_updates"),
        ],
    )

    # With SLAM disabled nothing publishes parrot1_map -> parrot1_odom, so the
    # map frame would not exist and nothing could be planned.  The EKF fuses the
    # exact Gazebo pose, so parrot1_odom already coincides with world; pin the
    # map frame to it with an identity static.  That keeps the chain
    # world -> parrot1_map -> parrot1_odom -> parrot1_base_link valid and the
    # sweep's planning frame drift-free (no scan-matching to wander off).
    map_to_odom_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=ROBOT_NAMESPACE,
        name="map_to_odom_static",
        output="screen",
        arguments=[
            "--frame-id", f"{ROBOT_NAMESPACE}_map",
            "--child-frame-id", f"{ROBOT_NAMESPACE}_odom",
        ],
        condition=UnlessCondition(slam_enabled),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            slam_arg,
            DeclareLaunchArgument("log_level", default_value="warn",
                                  description="Localisation logging level."),
            ekf,
            slam_toolbox,
            map_to_odom_static,
        ]
    )
