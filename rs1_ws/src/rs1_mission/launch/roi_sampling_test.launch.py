"""Launch only the parrot mission executor for the parrot-only ROI sampling test.

Use with a parrot-only simulation (`rs1_sim.launch.py husky:=false`) and the
perception stack, then run `test_roi_sampling.py`. No mission coordinator or
husky executor is started.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    log_level = LaunchConfiguration("log_level")

    launch_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    parrot_mission_executor = Node(
        package="rs1_mission",
        executable="parrot_mission_executor",
        namespace="parrot1",
        name="parrot_mission_executor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(launch_arguments + [parrot_mission_executor])
