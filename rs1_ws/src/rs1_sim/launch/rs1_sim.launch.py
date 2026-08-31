from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

"""
    Launches the gazebo world, spawns the husky and parrots in. Runs:
    - gz_world.launch.py
    - spawn_husky.launch.py
    - spawn_parrot.launch.py
    - husky_localisation.launch.py
    - parrot_localisation.launch.py
"""

RS1_SIM_PACKAGE = "rs1_sim"
RS1_LOCALISATION_PACKAGE = "rs1_localisation"
RS1_NAVIGATION_PACKAGE = "rs1_navigation"
LAUNCH_DIRECTORY = "launch"


def generate_launch_description():
    package_share = FindPackageShare(RS1_SIM_PACKAGE)
    localisation_package_share = FindPackageShare(RS1_LOCALISATION_PACKAGE)
    navigation_package_share = FindPackageShare(RS1_NAVIGATION_PACKAGE)

    # Launch configs
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    husky_enabled = LaunchConfiguration("husky")
    parrot_enabled = LaunchConfiguration("parrot")
    rviz_enabled = LaunchConfiguration("rviz")
    slam_enabled = LaunchConfiguration("slam")
    nav2_enabled = LaunchConfiguration("nav2")
    husky_spawn_delay = LaunchConfiguration("husky_spawn_delay")
    parrot_spawn_delay = LaunchConfiguration("parrot_spawn_delay")

    # Launch args
    launch_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value="v2_forest",
            choices=["large_demo", "simple_trees", "v1_forest", "v2_forest"],
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
            description="Seconds to wait before spawning the Husky.",
        ),
        DeclareLaunchArgument(
            "parrot_spawn_delay",
            default_value="2.0",
            description="Seconds to wait before spawning the Parrot.",
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

    # Launch gazebo world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    package_share,
                    LAUNCH_DIRECTORY,
                    "gz_world.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
        }.items(),
    )

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
            "use_sim_time": use_sim_time,
            "husky_spawn_delay": husky_spawn_delay,
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
            "use_sim_time": use_sim_time,
            "parrot_spawn_delay": parrot_spawn_delay,
        }.items(),
    )

    # RViz
    husky_rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="husky1",
        name="rviz2",
        condition=IfCondition(rviz_enabled),
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            PathJoinSubstitution([package_share, "config", "husky1.rviz"]),
        ],
        remappings=[
            ("/tf", "/husky1/tf"),
            ("/tf_static", "/husky1/tf_static"),
        ],
    )

    parrot_rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="parrot1",
        name="rviz2",
        condition=IfCondition(rviz_enabled),
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            PathJoinSubstitution([package_share, "config", "parrot1.rviz"]),
        ],
        remappings=[
            ("/tf", "/parrot1/tf"),
            ("/tf_static", "/parrot1/tf_static"),
        ],
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
        }.items(),
    )

    # Define groups
    world_group = GroupAction(
        scoped=True,
        actions=[world_launch],
    )

    husky_group = GroupAction(
        scoped=True,
        condition=IfCondition(husky_enabled),
        actions=[
            husky_launch,
            husky_rviz,
            husky_localisation_launch,
            husky_navigation_launch,
        ],
    )

    parrot_group = GroupAction(
        scoped=True,
        condition=IfCondition(parrot_enabled),
        actions=[
            parrot_launch,
            parrot_rviz,
            parrot_localisation_launch,
            parrot_navigation_launch,
        ],
    )

    actions = launch_arguments + [
        world_group,
        husky_group,
        parrot_group,
    ]

    return LaunchDescription(actions)
