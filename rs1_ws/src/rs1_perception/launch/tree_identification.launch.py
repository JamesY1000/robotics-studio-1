from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file= os.path.join(
        get_package_share_directory('rs1_perception'),
        'config',
        'yolo.yaml'
    )

    return LaunchDescription([
        Node(
            package='rs1_perception',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[config_file]
        )
    ])