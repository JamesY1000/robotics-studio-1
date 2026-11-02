"""Run one controller driver and Twist converter for both robots"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = get_package_share_directory('rs1_navigation') + '/config/ps4_teleop.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('device_name', default_value=''),
        # Use game_controller_node from joy for SDL's consistent PS4 mapping
        Node(
            package='joy', executable='game_controller_node', name='joy_node',
            namespace='teleop', output='screen', parameters=[config, {
                'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                'device_name': ParameterValue(LaunchConfiguration('device_name'), value_type=str),
            }],
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            namespace='teleop', output='screen', parameters=[config],
            remappings=[('cmd_vel', 'cmd_vel_controller')],
        ),
    ])
