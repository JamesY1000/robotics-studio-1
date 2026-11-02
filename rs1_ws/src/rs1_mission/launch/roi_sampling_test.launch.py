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
    max_fallback_hops = ParameterValue(
        LaunchConfiguration("max_fallback_hops"), value_type=int)
    waypoint_stall_timeout = ParameterValue(
        LaunchConfiguration("waypoint_stall_timeout"), value_type=float)

    launch_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
        # How many exploration round-trips to spend on ONE waypoint before
        # skipping it.  1 (default) explores once, then skips; 0 skips at once.
        DeclareLaunchArgument("max_fallback_hops", default_value="1"),
        # Seconds with no movement before the current waypoint is skipped.
        DeclareLaunchArgument("waypoint_stall_timeout", default_value="60.0"),
    ]

    parrot_mission_executor = Node(
        package="rs1_mission",
        executable="parrot_mission_executor",
        namespace="parrot1",
        name="parrot_mission_executor_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "max_fallback_hops": max_fallback_hops,
            "waypoint_stall_timeout": waypoint_stall_timeout,
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(launch_arguments + [parrot_mission_executor])
