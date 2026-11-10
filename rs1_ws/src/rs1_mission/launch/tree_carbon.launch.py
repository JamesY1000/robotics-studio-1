"""Standalone AGB estimates for an existing mission survey"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rs1_mission', executable='carbon_calculator_node.py',
             name='carbon_calculator', output='screen', parameters=[
                 PathJoinSubstitution([FindPackageShare('rs1_mission'), 'config', 'tree_carbon.yaml']),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
             ]),
    ])
