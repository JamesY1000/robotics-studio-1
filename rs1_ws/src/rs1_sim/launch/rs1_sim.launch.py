"""Start a complete simulation, or only its world for dashboard deployment."""
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rs1_sim")
    world = LaunchConfiguration("world")
    session_id = LaunchConfiguration("session_id")
    ui = LaunchConfiguration("ui")

    def include(file, arguments, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([package_share, "launch", file])
            ),
            launch_arguments=arguments.items(),
            condition=condition,
        )

    world_launch = include("gz_world.launch.py", {
        "world": world,
        "world_file": LaunchConfiguration("world_file"),
        "headless": LaunchConfiguration("headless"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "log_level": LaunchConfiguration("stack_log_level"),
    })
    robot_launch = include("robot_bringup.launch.py", {
        "world": world,
        "session_id": session_id,
        "combined": LaunchConfiguration("combined"),
    }, condition=UnlessCondition(ui))
    ui_rviz = include("rviz.launch.py", {
        "combined": LaunchConfiguration("combined"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "stack_log_level": LaunchConfiguration("stack_log_level"),
    }, condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription([
        DeclareLaunchArgument(
            "ui", default_value="False",
            description="Skip robot bringup; deploy later from the dashboard. RViz can be enabled separately.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value=PythonExpression(["'", ui, "'.lower() not in ('true', '1')"]),
            description="Open robot RViz displays; defaults to true normally, false with ui:=true.",
        ),
        DeclareLaunchArgument(
            "combined", default_value="False",
            description="Use one shared RViz window when RViz is enabled.",
        ),
        DeclareLaunchArgument(
            "world", default_value="v2_forest",
            choices=["large_demo", "simple_trees", "v1_forest", "v2_forest", "v2_forest_labeled"],
            description="Gazebo world to load.",
        ),
        DeclareLaunchArgument("world_file", default_value=""),
        DeclareLaunchArgument("headless", default_value="False"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("stack_log_level", default_value="warn"),
        DeclareLaunchArgument("session_id", default_value="manual"),
        GroupAction(scoped=True, actions=[world_launch]),
        Node(
            package="rs1_sim", executable="simulation_session.py",
            name="simulation_session", output="screen",
            parameters=[{
                "use_sim_time": False,
                "mode": "publish",
                "instance_id": str(uuid4()),
                "world_name": ParameterValue(world, value_type=str),
                "world_file": ParameterValue(LaunchConfiguration("world_file"), value_type=str),
                "ui": ParameterValue(ui, value_type=bool),
                "session_id": ParameterValue(session_id, value_type=str),
            }],
        ),
        GroupAction(scoped=True, actions=[robot_launch]),
        GroupAction(scoped=True, condition=IfCondition(ui), actions=[ui_rviz]),
    ])
