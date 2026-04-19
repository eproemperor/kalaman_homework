from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    current_file = Path(__file__).resolve()
    workspace_root = current_file.parent.parent.parent  
    params_file = str(workspace_root / 'src' / 'config' / 'params.yaml')
    
    print(f"Workspace root: {workspace_root}")
    print(f"Params file: {params_file}")
    
    return LaunchDescription([
        Node(
            package='vision_kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='serial_comman',
            executable='serial_comman_node',
            name='serial_comman_node',
            parameters=[params_file],
            output='screen'
        )
    ])