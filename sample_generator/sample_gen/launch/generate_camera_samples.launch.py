import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Directory that contains the tree models so `model://trees/...` mesh URIs resolve.
# This should be `rs1_ws/src/rs1_sim/models` (the parent of the trees/ folder).
MODEL_PATH = "/home/tim/git/robotics-studio-1/rs1_ws/src/rs1_sim/models"

ARGUMENTS = [
    DeclareLaunchArgument(
        "world",
        default_value="simple_trees",
        description="World to launch simulation with",
    ),
]


def _append_env(name: str, extra: str) -> SetEnvironmentVariable:
    """Append ``extra`` (colon separated) to an existing environment variable."""
    current = os.environ.get(name, "")
    merged = extra if not current else f"{current}:{extra}"
    return SetEnvironmentVariable(name, merged)


def generate_launch_description():
    bringup_pkg_dir = get_package_share_directory("41068_ignition_bringup")
    sample_gen_pkg_dir = get_package_share_directory("sample_gen")
    plugin_pkg_dir = get_package_share_directory("sample_gen_gazebo_plugins")

    ld = LaunchDescription(ARGUMENTS)

    # Make the local tree models + the object_label plugin visible to Gazebo
    ld.add_action(_append_env("IGN_GAZEBO_RESOURCE_PATH", MODEL_PATH))
    ld.add_action(_append_env("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", plugin_pkg_dir))

    # Parrot-only bringup (robot_state_publisher + robot_localization + bridges)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_pkg_dir, "launch", "41068_ignition.launch.py"])
        ),
        launch_arguments=[
            ("world", LaunchConfiguration("world")),
            ("husky", "False"),
            ("parrot", "True"),
            ("slam", "False"),
            ("nav2", "False"),
            ("rviz", "False"),
        ],
    )

    config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "config.yml"])

    camera_collector_node = Node(
        package="sample_gen",
        name="camera_collector_node",
        executable="camera_collector.py",
        parameters=[config, {"use_sim_time": True}],
        output="screen",
    )

    model_tf_broadcaster_node = Node(
        package="sample_gen",
        name="model_tf_broadcaster_node",
        executable="model_tf_broadcaster.py",
        on_exit=Shutdown(),
        output="screen",
    )

    parrot_world_broadcaster_node = Node(
        package="sample_gen",
        name="parrot_world_broadcaster_node",
        executable="parrot_world_broadcaster.py",
        parameters=[config, {"use_sim_time": True}],
        on_exit=Shutdown(),
        output="screen",
    )

    sample_generator_node = Node(
        package="sample_gen",
        name="sample_generator_node",
        executable="sample_generator.py",
        parameters=[config, {"use_sim_time": True}],
        on_exit=Shutdown(),
        output="screen",
    )

    ld.add_action(sim)
    ld.add_action(parrot_world_broadcaster_node)
    ld.add_action(model_tf_broadcaster_node)
    ld.add_action(camera_collector_node)
    ld.add_action(sample_generator_node)

    return ld


if __name__ == "__main__":
    generate_launch_description()
