import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Directory that contains the tree models (originals + the *_labeled copies
# that carry the ObjectLabelPlugin) so `model://trees/...` mesh URIs resolve.
# This should be `rs1_ws/src/rs1_sim/models` (the parent of the trees/ folder).
MODEL_PATH = "/home/tim/git/robotics-studio-1/rs1_ws/src/rs1_sim/models"

ARGUMENTS = [
    DeclareLaunchArgument(
        "world",
        default_value="v2_forest_labeled",
        description="rs1_sim world to load (v2_forest_labeled = trees carry the ObjectLabelPlugin)",
    ),
]


def _append_env(name: str, extra: str) -> SetEnvironmentVariable:
    """Append ``extra`` (colon separated) to an existing environment variable."""
    current = os.environ.get(name, "")
    merged = extra if not current else f"{current}:{extra}"
    return SetEnvironmentVariable(name, merged)


def generate_launch_description():
    rs1_sim_pkg_dir = get_package_share_directory("rs1_sim")
    sample_gen_pkg_dir = get_package_share_directory("sample_gen")
    plugin_pkg_dir = get_package_share_directory("sample_gen_gazebo_plugins")

    ld = LaunchDescription(ARGUMENTS)

    # Make the local tree models + the object_label plugin visible to Gazebo.
    ld.add_action(_append_env("IGN_GAZEBO_RESOURCE_PATH", MODEL_PATH))
    ld.add_action(_append_env("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", plugin_pkg_dir))

    # rs1_ws environment with just the Parrot (no Husky / SLAM / Nav2 / RViz).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rs1_sim_pkg_dir, "launch", "rs1_sim.launch.py"])
        ),
        launch_arguments=[
            ("world", LaunchConfiguration("world")),
            ("husky", "false"),
            ("parrot", "true"),
            ("slam", "false"),
            ("nav2", "false"),
            ("rviz", "false"),
        ],
    )

    config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "config.yml"])

    camera_collector_node = Node(
        package="sample_gen",
        name="camera_collector_node",
        executable="camera_collector.py",
        parameters=[config, {"use_sim_time": True}, {"oak_z_center_offset": 0.65}],
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

    parrot_patrol_node = Node(
        package="sample_gen",
        name="parrot_patrol_node",
        executable="parrot_patrol.py",
        parameters=[config, {"use_sim_time": True}],
        on_exit=Shutdown(),
        output="screen",
    )

    ld.add_action(sim)
    ld.add_action(parrot_world_broadcaster_node)
    ld.add_action(model_tf_broadcaster_node)
    ld.add_action(camera_collector_node)
    ld.add_action(parrot_patrol_node)

    return ld


if __name__ == "__main__":
    generate_launch_description()
