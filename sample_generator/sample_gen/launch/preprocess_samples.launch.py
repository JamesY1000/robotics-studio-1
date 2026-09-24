import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit


ARGUMENTS = [
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        choices=["true", "false"],
        description="Start rviz.",
    ),
]


def generate_launch_description():
    sample_gen_pkg_dir = get_package_share_directory("sample_gen")

    rviz_config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "sample_gen.rviz"])
    config = PathJoinSubstitution([sample_gen_pkg_dir, "config", "config.yml"])

    rviz = Node(
        package="rviz2",
        name="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    data_preprocessor_node = Node(
        package="sample_gen",
        name="data_preprocessor_node",
        executable="data_preprocessor.py",
        parameters=[config, {"use_sim_time": True}],
        output="screen",
    )

    model_tf_broadcaster_node = Node(
        package="sample_gen",
        name="model_tf_broadcaster_node",
        executable="model_tf_broadcaster.py",
        output="screen",
    )

    robot_world_broadcaster = Node(
        package="sample_gen",
        name="robot_world_broadcaster_node",
        executable="robot_world_broadcaster.py",
        output="screen",
    )

    yolo_format_node = Node(
        package="sample_gen",
        name="yolo_format_node",
        executable="yolo_format.py",
        on_exit=launch.actions.Shutdown(),
        output="screen",
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(model_tf_broadcaster_node)
    ld.add_action(robot_world_broadcaster)
    ld.add_action(rviz)
    ld.add_action(data_preprocessor_node)

    # Run yolo_format_node once data_preprocessor node has finished
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=data_preprocessor_node,
                on_exit=[yolo_format_node],
            )
        )
    )

    return ld


if __name__ == "__main__":
    generate_launch_description()
