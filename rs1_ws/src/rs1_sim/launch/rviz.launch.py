"""Open the robot displays independently of robot spawning and navigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def start_rviz(context):
    robots = [
        robot
        for robot in ("husky", "parrot")
        if IfCondition(LaunchConfiguration(robot)).evaluate(context)
    ]
    if not robots:
        return []

    # A single shared view uses global TF and the common world frame. Keep the
    # original per-robot windows available for detailed navigation debugging.
    combined = IfCondition(LaunchConfiguration("combined")).evaluate(context)
    views = ["combined"] if combined else [f"{robot}1" for robot in robots]
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            namespace="/" if combined else view,
            name="rviz2",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                }
            ],
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("rs1_sim"), "config", f"{view}.rviz"]
                ),
                "--ros-args",
                "--log-level",
                LaunchConfiguration("stack_log_level"),
            ],
            # Use global TF: alignment aggregates the robot trees here and
            # adds world -> robot_map, needed by the world-frame ROI marker.
        )
        for view in views
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("husky", default_value="True"),
            DeclareLaunchArgument("parrot", default_value="True"),
            DeclareLaunchArgument(
                "combined",
                default_value="True",
                description="Show both robots in one world-frame RViz window.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("stack_log_level", default_value="warn"),
            OpaqueFunction(function=start_rviz),
        ]
    )
