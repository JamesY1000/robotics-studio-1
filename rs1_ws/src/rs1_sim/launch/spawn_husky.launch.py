from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

"""
    Handles rsp, ros_gz bridge, and spawning of husky
"""

# Include namespace and frame prefix
ROBOT_NAMESPACE = "husky1"
FRAME_PREFIX = "husky1_"
GZ_MODEL_NAME = "husky1"

SPAWN_X = "95.0"
SPAWN_Y = "-45.0"
SPAWN_Z = "6.0"
SPAWN_YAW = "0.0"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_delay = LaunchConfiguration("husky_spawn_delay")

    description_path = PathJoinSubstitution(
        [
            FindPackageShare("rs1_description"),
            "urdf",
            "husky",
            "husky.urdf.xacro",
        ]
    )
    bridge_config = PathJoinSubstitution(
        [
            FindPackageShare("rs1_sim"),
            "config",
            "gazebo_bridge_husky1.yml",
        ]
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_path,
                " prefix:=",
                FRAME_PREFIX,
                " gz_model_name:=",
                GZ_MODEL_NAME,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ROBOT_NAMESPACE,
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=ROBOT_NAMESPACE,
        name="gazebo_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_husky = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_husky1",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-topic",
            f"/{ROBOT_NAMESPACE}/robot_description",
            "-name",
            GZ_MODEL_NAME,
            "-x",
            SPAWN_X,
            "-y",
            SPAWN_Y,
            "-z",
            SPAWN_Z,
            "-Y",
            SPAWN_YAW,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("husky_spawn_delay", default_value="3.0"),
            robot_state_publisher,
            gazebo_bridge,
            TimerAction(
                period=spawn_delay,
                actions=[spawn_husky],
            ),
        ]
    )
