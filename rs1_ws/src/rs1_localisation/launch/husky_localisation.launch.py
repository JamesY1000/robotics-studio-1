from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
Handles launching localisation and mapping nodes for husky
"""

PKG_NAME = "rs1_localisation"
ROBOT_NAMESPACE = "husky1"
ROBOT = "husky"
EKF_CONFIG = "robot_localization_husky1.yml"
SLAM_CONFIG = "slam_params_husky1.yml"


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

    # Convert body-frame gyro rates to the horizontal heading rate expected by
    # the 2D EKF. This does not modify the raw IMU used by LiDAR leveling.
    heading_rate = Node(
        package=PKG_NAME,
        executable="husky_heading_rate.py",
        namespace=ROBOT_NAMESPACE,
        name="husky_heading_rate",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
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

    return LaunchDescription(
        [
            use_sim_time_arg,
            slam_arg,
            DeclareLaunchArgument("log_level", default_value="warn",
                                  description="Localisation logging level."),
            DeclareLaunchArgument("ground_filter", default_value="true",
                                  description="Start Husky's terrain filtering pipeline."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("rs1_perception"), "launch", "husky_ground.launch.py"])),
                condition=IfCondition(LaunchConfiguration("ground_filter")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            heading_rate,
            ekf,
            slam_toolbox,
        ]
    )
