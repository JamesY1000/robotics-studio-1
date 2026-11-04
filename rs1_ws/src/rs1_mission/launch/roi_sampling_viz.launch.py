"""Start the ROI sampling RViz view (ROI bounds, found trees, ground truth).

Run alongside the parrot-only simulation and the ROI sampling test.

Note: RViz may fail to start from a sandboxed/snap terminal (GLIBC clash); in
that case start `rviz2` yourself with the printed config path.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    launch_arguments = [
        DeclareLaunchArgument("roi_x_min", default_value="62.0"),
        DeclareLaunchArgument("roi_y_min", default_value="-88.0"),
        DeclareLaunchArgument("roi_x_max", default_value="92.0"),
        DeclareLaunchArgument("roi_y_max", default_value="-58.0"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("rviz", default_value="True"),
    ]

    viz = Node(
        package="rs1_mission",
        executable="roi_sampling_viz.py",
        name="roi_sampling_viz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--roi",
            LaunchConfiguration("roi_x_min"),
            LaunchConfiguration("roi_y_min"),
            LaunchConfiguration("roi_x_max"),
            LaunchConfiguration("roi_y_max"),
        ],
    )

    config = Path(get_package_share_directory("rs1_mission")) / "config" / "roi_sampling.rviz"
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", str(config)],
    )

    return LaunchDescription(launch_arguments + [viz, rviz2])
