from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, RosTimer, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

"""
    Handles rsp, ros_gz bridge, and spawning of husky
"""

# Include namespace and frame prefix
ROBOT_NAMESPACE = "husky1"
FRAME_PREFIX = "husky1_"
GZ_MODEL_NAME = "husky1"

# Gazebo coords, not slam
SPAWN_X = "90.0"
SPAWN_Y = "-48.0"
SPAWN_Z = "6.0"
SPAWN_YAW = "0.0"


def start_robot(context):
    # The delayed create action executes after its including scope is popped.
    def value(name):
        return LaunchConfiguration(name).perform(context)

    use_sim_time = value("use_sim_time").lower() in ("true", "1")
    spawn_delay = value("husky_spawn_delay")
    log_level = value("log_level")

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

    spawn_husky = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_husky1",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-world",
            value("world"),
            "-topic",
            f"/{ROBOT_NAMESPACE}/robot_description",
            "-name",
            GZ_MODEL_NAME,
            "-allow_renaming=false",
            "-x",
            value("husky_x"),
            "-y",
            value("husky_y"),
            "-z",
            value("husky_z"),
            "-Y",
            value("husky_yaw"),
            "--ros-args",
            "--log-level",
            log_level,
        ],
    )

    return [
        # A cold world can take longer to load than create's service timeout.
        # Simulation time only advances once Gazebo is running the world.
        *([SetUseSimTime(True)] if use_sim_time else []),
        robot_state_publisher,
        gazebo_bridge,
        (RosTimer if use_sim_time else TimerAction)(
            period=float(spawn_delay),
            actions=[spawn_husky],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="v2_forest"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="Robot spawning and bridge logging level.",
            ),
            DeclareLaunchArgument(
                "husky_spawn_delay", default_value="3.0",
                description="Delay in simulation seconds (wall seconds if use_sim_time is false).",
            ),
            DeclareLaunchArgument("husky_x", default_value=SPAWN_X),
            DeclareLaunchArgument("husky_y", default_value=SPAWN_Y),
            DeclareLaunchArgument("husky_z", default_value=SPAWN_Z),
            DeclareLaunchArgument("husky_yaw", default_value=SPAWN_YAW),
            OpaqueFunction(function=start_robot),
        ]
    )
