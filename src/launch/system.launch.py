from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.path.expanduser('~/ros2_ws/src/config/params.yaml')
    
    return LaunchDescription([
        Node(
            package='vision_kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            parameters=[params_file] 
        ),
        Node(
            package='serial_comman',
            executable='serial_comman_node',
            name='serial_comman_node',
            parameters=[params_file]
        )
    ])