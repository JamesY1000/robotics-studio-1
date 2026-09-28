"""One-command visual check of the YOLO tree-detection -> TreeLocation pipeline.

Requires the shared aligned TF tree. Teleport tours invalidate localization;
use stationary views with tour:=false for an aligned mission session.

Starts:

* the perception stack (``tree_identification.launch.py``: shared world alignment required;
  YOLO node + tree-location node),
* RViz with ``config/tree_locations.rviz`` (fixed frame ``world``, MarkerArray
  on ``/tree_locations/markers``, the YOLO debug image and the parrot lidar),
* ``visualise_tree_locations.py``, which runs a detect -> process cycle and
  drops a coloured marker per identified tree (one colour per tree type), and
  (``gazebo:=true``, the default) spawns a matching marker pole in the Gazebo
  world so the result can be checked against the real trees in the sim view.

Requires the simulation to be running already (same ROS domain); it includes shared alignment::

    ros2 launch rs1_sim rs1_sim.launch.py husky:=false rviz:=false

Then::

    ros2 launch rs1_perception tree_locations_viz.launch.py
    # Teleport tours are disabled by default; they invalidate localization.
    # show every true tree position as a grey RViz marker too:
    ros2 launch rs1_perception tree_locations_viz.launch.py show_ground_truth:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rs1_perception")
    perception_launch = PathJoinSubstitution(
        [package_share, "launch", "tree_identification.launch.py"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "config", "tree_locations.rviz"]
    )

    tour = LaunchConfiguration("tour")
    views = LaunchConfiguration("views")
    interval = LaunchConfiguration("interval")
    show_ground_truth = LaunchConfiguration("show_ground_truth")
    gazebo = LaunchConfiguration("gazebo")
    step = LaunchConfiguration("step")
    rviz = LaunchConfiguration("rviz")

    # argparse flags: translate the launch booleans into --tour / --no-tour etc.
    tour_flag = PythonExpression(
        ["'--tour' if '", tour, "' == 'true' else '--no-tour'"]
    )
    gt_flag = PythonExpression(
        [
            "'--show-ground-truth' if '", show_ground_truth,
            "' == 'true' else '--no-show-ground-truth'",
        ]
    )
    gazebo_flag = PythonExpression(
        ["'--gazebo' if '", gazebo, "' == 'true' else '--no-gazebo'"]
    )
    step_flag = PythonExpression(
        ["'--step' if '", step, "' == 'true' else '--no-step'"]
    )

    arguments = [
        DeclareLaunchArgument("tour", default_value="false",
                              description="Legacy teleport tour; invalidates registered localization."),
        DeclareLaunchArgument("views", default_value="6",
                              description="Number of tour stops."),
        DeclareLaunchArgument("interval", default_value="5.0",
                              description="Seconds between refreshes."),
        DeclareLaunchArgument("step", default_value="false",
                              description="Wait for Enter between stops. Only useful when the "
                                          "visualiser has an interactive terminal, i.e. run it "
                                          "with `ros2 run` instead of this launch file."),
        DeclareLaunchArgument("show_ground_truth", default_value="true",
                              description="Draw a grey marker at every true tree position."),
        DeclareLaunchArgument("gazebo", default_value="true",
                              description="Spawn coloured marker poles in the Gazebo world."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz with the tree-locations config."),
    ]

    # The perception stack (shared world alignment required; yolo_node + tree_location_node).
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    visualiser = Node(
        package="rs1_perception",
        executable="visualise_tree_locations.py",
        name="tree_location_visualiser",
        output="screen",
        arguments=[
            tour_flag,
            "--views", views,
            "--interval", interval,
            "--settle-time", "3.0",
            gt_flag,
            gazebo_flag,
            step_flag,
            "--no-rviz",
        ],
    )

    return LaunchDescription(
        arguments
        + [
            perception,
            rviz_node,
            # Give the sim bridges/services a moment to come up before calling them.
            TimerAction(period=4.0, actions=[visualiser]),
        ]
    )
