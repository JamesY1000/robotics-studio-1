from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch_ros.actions import (
    Node,
    RosTimer,
    SetUseSimTime,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

"""
    Spawns both robots and starts their supporting nodes in an existing world.
    - spawn_husky.launch.py
    - spawn_parrot.launch.py
    - husky_localisation.launch.py
    - parrot_localisation.launch.py
    - shared world/map alignment (once per simulation)
"""

RS1_SIM_PACKAGE = "rs1_sim"
RS1_LOCALISATION_PACKAGE = "rs1_localisation"
RS1_NAVIGATION_PACKAGE = "rs1_navigation"
LAUNCH_DIRECTORY = "launch"


def start_robots(context):
    # Humble timers retain context locals, but not scoped launch arguments.
    # Resolve every value before the including GroupAction restores its scope.
    def value(name):
        return LaunchConfiguration(name).perform(context)

    package_share = FindPackageShare(RS1_SIM_PACKAGE)
    localisation_package_share = FindPackageShare(RS1_LOCALISATION_PACKAGE)
    navigation_package_share = FindPackageShare(RS1_NAVIGATION_PACKAGE)

    # Launch configs
    world = value("world")
    use_sim_time = value("use_sim_time")
    use_sim_time_bool = use_sim_time.lower() in ("true", "1")
    husky_enabled = value("husky")
    parrot_enabled = value("parrot")
    rviz_enabled = value("rviz")
    slam_enabled = value("slam")
    nav2_enabled = value("nav2")
    husky_spawn_delay = value("husky_spawn_delay")
    parrot_spawn_delay = value("parrot_spawn_delay")
    log_level = value("log_level")
    stack_log_level = value("stack_log_level")

    # Spawn husky
    husky_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    LAUNCH_DIRECTORY,
                    "spawn_husky.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "husky_spawn_delay": husky_spawn_delay,
            "husky_x": value("husky_x"),
            "husky_y": value("husky_y"),
            "husky_z": value("husky_z"),
            "husky_yaw": value("husky_yaw"),

            "log_level": stack_log_level,
        }.items(),
    )

    # Spawn parrot
    parrot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    LAUNCH_DIRECTORY,
                    "spawn_parrot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "parrot_spawn_delay": parrot_spawn_delay,
            "parrot_x": value("parrot_x"),
            "parrot_y": value("parrot_y"),
            "parrot_z": value("parrot_z"),
            "parrot_yaw": value("parrot_yaw"),

            "log_level": stack_log_level,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([package_share, LAUNCH_DIRECTORY, "rviz.launch.py"])),
        condition=IfCondition(rviz_enabled),
        launch_arguments={
            "husky": husky_enabled, "parrot": parrot_enabled,
            "use_sim_time": use_sim_time, "stack_log_level": stack_log_level,
        }.items(),
    )

    husky_localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    localisation_package_share,
                    "launch",
                    "husky_localisation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": slam_enabled,
            "log_level": stack_log_level,
        }.items(),
    )

    parrot_localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    localisation_package_share,
                    "launch",
                    "parrot_localisation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": slam_enabled,
            "log_level": stack_log_level,
        }.items(),
    )

    husky_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    navigation_package_share,
                    "launch",
                    "husky_navigation.launch.py",
                ]
            )
        ),
        condition=IfCondition(nav2_enabled),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": stack_log_level,
        }.items(),
    )

    parrot_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    navigation_package_share,
                    "launch",
                    "parrot_navigation.launch.py",
                ]
            )
        ),
        condition=IfCondition(nav2_enabled),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "nav2_log_level": stack_log_level,
        }.items(),
    )

    # Define groups
    husky_group = GroupAction(
        scoped=True,
        condition=IfCondition(husky_enabled),
        actions=[
            GroupAction(scoped=True, actions=[husky_launch]),
            # Introduce a delay as gazebo interprets a massive velocity spike if robot is spawned away from (0,0)
            # and this is getting accumulated into ekf
            RosTimer(
                period=3.0,  # Should work under current simulation load, adjust if necessary
                actions=[
                    GroupAction(
                        scoped=True,
                        actions=[
                            GroupAction(scoped=True, actions=[husky_localisation_launch]),
                            GroupAction(scoped=True, actions=[husky_navigation_launch]),
                        ],
                    )
                ],
            ),
        ],
    )

    parrot_group = GroupAction(
        scoped=True,
        condition=IfCondition(parrot_enabled),
        actions=[
            GroupAction(scoped=True, actions=[parrot_launch]),
            # Introduce a delay as gazebo interprets a massive velocity spike if robot is spawned away from (0,0)
            # and this is getting accumulated into ekf
            RosTimer(
                period=3.0,  # Should work under current simulation load, adjust if necessary
                actions=[
                    GroupAction(
                        scoped=True,
                        actions=[
                            GroupAction(scoped=True, actions=[parrot_localisation_launch]),
                            GroupAction(scoped=True, actions=[parrot_navigation_launch]),
                        ],
                    )
                ],
            ),
        ],
    )

    actions = [
        SetUseSimTime(use_sim_time_bool),
        GroupAction(scoped=True, actions=[rviz_launch]),
        husky_group,
        parrot_group,
        Node(
            package="rs1_alignment",
            executable="alignment",
            name="mission_alignment",
            output="screen",
            condition=IfCondition(value("alignment")),
            parameters=[{
                "use_sim_time": use_sim_time_bool,
                "session_id": ParameterValue(value("session_id"), value_type=str),
                "world_name": world,
            }],
            arguments=["--ros-args", "--log-level", stack_log_level],
        ),
    ]

    return actions


def generate_launch_description():
    # Launch args
    launch_arguments = [
        DeclareLaunchArgument(
            "alignment",
            default_value="True",
            description="Start shared world/map alignment and global TF aggregation.",
        ),
        DeclareLaunchArgument(
            "session_id",
            default_value="manual",
            description="Simulation session identifier; match the mission launch.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Exploration and terrain-following logging level.",
        ),
        DeclareLaunchArgument(
            "stack_log_level",
            default_value="warn",
            description="Nav2, localisation, RViz, and bridge logging level.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="v2_forest",
            choices=["large_demo", "simple_trees", "v1_forest", "v2_forest", "v2_forest_labeled"],
            description="Gazebo world to load.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "husky",
            default_value="True",
            description="Spawn the Husky robot.",
        ),
        DeclareLaunchArgument(
            "parrot",
            default_value="True",
            description="Spawn the Parrot robot.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="True",
            description="Open one RViz window for each enabled robot.",
        ),
        DeclareLaunchArgument(
            "husky_spawn_delay",
            default_value="2.0",
            description="Simulation seconds before spawning the Husky (wall seconds if use_sim_time is false).",
        ),
        DeclareLaunchArgument(
            "parrot_spawn_delay",
            default_value="2.0",
            description="Simulation seconds before spawning the Parrot (wall seconds if use_sim_time is false).",
        ),
        DeclareLaunchArgument(
            "slam",
            default_value="True",
            description="Start SLAM toolbox",
        ),
        DeclareLaunchArgument(
            "nav2",
            default_value="True",
            description="Start Nav2 for each enabled robot.",
        ),
    ]

    for robot, z in (("husky", "6.0"), ("parrot", "8.0")):
        for axis, default in (("x", "90.0"), ("y", "-48.0"), ("z", z), ("yaw", "0.0")):
            launch_arguments.append(DeclareLaunchArgument(f"{robot}_{axis}", default_value=default))

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=start_robots)])
