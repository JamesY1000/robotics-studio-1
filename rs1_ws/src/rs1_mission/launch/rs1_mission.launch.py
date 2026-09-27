"""Launch the mission coordinator and both robot mission executors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG_NAME = "rs1_mission"


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    log_level = LaunchConfiguration("log_level")

    launch_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "roi_x",
            default_value="-40.0",
            description="ROI x coordinate in parrot1_map (metres).",
        ),
        DeclareLaunchArgument(
            "roi_y",
            default_value="40.0",
            description="ROI y coordinate in parrot1_map (metres).",
        ),
        DeclareLaunchArgument(
            "roi_z",
            default_value="0.0",
            description="ROI z coordinate in parrot1_map (metres).",
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
                "roi_x": ParameterValue(LaunchConfiguration("roi_x"), value_type=float),
                "roi_y": ParameterValue(LaunchConfiguration("roi_y"), value_type=float),
                "roi_z": ParameterValue(LaunchConfiguration("roi_z"), value_type=float),
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
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    husky_mission_executor = Node(
        package=PKG_NAME,
        executable="husky_mission_executor",
        namespace="husky1",
        name="husky_mission_executor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return LaunchDescription(
        launch_arguments
        + [mission_coordinator, parrot_mission_executor, husky_mission_executor]
    )
