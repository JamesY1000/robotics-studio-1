from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rs1_mission")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "csv_path",
                default_value=PathJoinSubstitution([pkg, "res", "tree_locations.csv"]),
            ),
            Node(
                package="rs1_mission",
                executable="carbon_calculator_node.py",
                name="carbon_calculator",
                output="screen",
                parameters=[
                    PathJoinSubstitution([pkg, "config", "tree_carbon.yaml"]),
                    {
                        "csv_path": LaunchConfiguration("csv_path")
                    },  # later entry overrides YAML
                ],
            ),
        ]
    )
