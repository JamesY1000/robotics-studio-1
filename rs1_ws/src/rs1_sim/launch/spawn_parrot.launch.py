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
    Handles rsp, ros_gz bridge, and spawning of parrot
"""

# Include namespace and frame prefix
ROBOT_NAMESPACE = "parrot1"
FRAME_PREFIX = "parrot1_"
GZ_MODEL_NAME = "parrot1"

# Gazebo coords, not slam
SPAWN_X = "90.0"
SPAWN_Y = "-48.0"
SPAWN_Z = "8.0"
SPAWN_YAW = "0.0"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_delay = LaunchConfiguration("parrot_spawn_delay")
    log_level = LaunchConfiguration("log_level")

    description_path = PathJoinSubstitution(
        [
            FindPackageShare("rs1_description"),
            "urdf",
            "parrot",
            "parrot.urdf.xacro",
        ]
    )
    bridge_config = PathJoinSubstitution(
        [
            FindPackageShare("rs1_sim"),
            "config",
            "gazebo_bridge_parrot1.yml",
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
        arguments=["--ros-args", "--log-level", log_level],
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
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_parrot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_parrot1",
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
            "--ros-args", "--log-level", log_level,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("log_level", default_value="warn",
                                  description="Robot spawning and bridge logging level."),
            DeclareLaunchArgument("parrot_spawn_delay", default_value="6.0"),
            robot_state_publisher,
            gazebo_bridge,
            TimerAction(
                period=spawn_delay,
                actions=[spawn_parrot],
            ),
        ]
    )
