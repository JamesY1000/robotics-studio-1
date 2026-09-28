"""Open the robot displays independently of robot spawning and navigation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("husky", default_value="True"),
        DeclareLaunchArgument("parrot", default_value="True"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("stack_log_level", default_value="warn"),
        *[
            Node(
                package="rviz2", executable="rviz2", namespace=f"{robot}1", name="rviz2",
                condition=IfCondition(LaunchConfiguration(robot)), output="screen",
                parameters=[{
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                }],
                arguments=[
                    "-d", PathJoinSubstitution([FindPackageShare("rs1_sim"), "config", f"{robot}1.rviz"]),
                    "--ros-args", "--log-level", LaunchConfiguration("stack_log_level"),
                ],
                # Use global TF: alignment aggregates the robot trees here and
                # adds world -> robot_map, needed by the world-frame ROI marker.
            )
            for robot in ("husky", "parrot")
        ],
    ])
