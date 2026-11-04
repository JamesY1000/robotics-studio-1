"""Launch the mission coordinator and both robot mission executors."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG_NAME = "rs1_mission"


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    log_level = LaunchConfiguration("log_level")

    launch_arguments = [
        DeclareLaunchArgument("joystick", default_value="true"),
        DeclareLaunchArgument("joystick_device_id", default_value="0"),
        DeclareLaunchArgument("joystick_device_name", default_value=""),
        DeclareLaunchArgument("roi_frame", default_value="world"),
        DeclareLaunchArgument("session_id", default_value="manual"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "roi_x",
            default_value="-40.0",
            description="ROI x coordinate in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_y",
            default_value="40.0",
            description="ROI y coordinate in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_z",
            default_value="0.0",
            description="ROI z coordinate in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_x_min",
            default_value="-45.0",
            description="ROI box minimum x in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_x_max",
            default_value="-35.0",
            description="ROI box maximum x in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_y_min",
            default_value="35.0",
            description="ROI box minimum y in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "roi_y_max",
            default_value="45.0",
            description="ROI box maximum y in roi_frame (metres).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level for all mission nodes.",
        ),
    ]

    mission_coordinator = Node(
        package=PKG_NAME,
        executable="mission_coordinator",
        name="mission_coordinator_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "roi_frame": LaunchConfiguration("roi_frame"),
                "session_id": LaunchConfiguration("session_id"),
                "roi_x": ParameterValue(LaunchConfiguration("roi_x"), value_type=float),
                "roi_y": ParameterValue(LaunchConfiguration("roi_y"), value_type=float),
                "roi_z": ParameterValue(LaunchConfiguration("roi_z"), value_type=float),
                "roi_x_min": ParameterValue(LaunchConfiguration("roi_x_min"), value_type=float),
                "roi_x_max": ParameterValue(LaunchConfiguration("roi_x_max"), value_type=float),
                "roi_y_min": ParameterValue(LaunchConfiguration("roi_y_min"), value_type=float),
                "roi_y_max": ParameterValue(LaunchConfiguration("roi_y_max"), value_type=float),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    parrot_mission_executor = Node(
        package=PKG_NAME,
        executable="parrot_mission_executor",
        namespace="parrot1",
        name="parrot_mission_executor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    husky_mission_executor = Node(
        package=PKG_NAME,
        executable="husky_mission_executor",
        namespace="husky1",
        name="husky_mission_executor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Start once per mission stack, including deploy and managed dashboard launches
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("rs1_navigation") + "/launch/joystick.launch.py"),
        condition=IfCondition(LaunchConfiguration("joystick")),
        launch_arguments={
            "device_id": LaunchConfiguration("joystick_device_id"),
            "device_name": LaunchConfiguration("joystick_device_name"),
        }.items(),
    )

    return LaunchDescription(
        launch_arguments
        + [mission_coordinator, parrot_mission_executor, husky_mission_executor, joystick]
    )
