from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tree_carbon')
    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_path',
            default_value=PathJoinSubstitution([pkg, 'data', 'tree_locations.csv'])),
        Node(
            package='tree_carbon',
            executable='carbon_calculator_node.py',
            name='carbon_calculator',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg, 'config', 'tree_carbon.yaml']),
                {'csv_path': LaunchConfiguration('csv_path')},  # later entry overrides YAML
            ],
        ),
    ])
