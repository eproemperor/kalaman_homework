from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial',  
            default_value='/dev/pts/5',
            description='Serial port device'
        ),
        DeclareLaunchArgument(
            'Q_POS',  
            default_value='10.4',
        ),
        DeclareLaunchArgument(
            'Q_VEL',  
            default_value='7.6',
        ),
        DeclareLaunchArgument(
            'Q_ACC',  
            default_value='15.5',
        ),
        DeclareLaunchArgument(
            'COMPENSATION_FACTOR',  
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'VEL_COMPENSATION_FACTOR',  
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'ACC_COMPENSATION_FACTOR',  
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'MIN_TARGET_CONFIDENCE',  
            default_value='0.14',
        ),
        Node(
            package='vision_kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            parameters=[{
                'serial_port': LaunchConfiguration('serial'),
                'Q_POS': LaunchConfiguration('Q_POS'),
                'Q_VEL': LaunchConfiguration('Q_VEL'),
                'Q_ACC': LaunchConfiguration('Q_ACC'),
                'COMPENSATION_FACTOR': LaunchConfiguration('COMPENSATION_FACTOR'),
                'VEL_COMPENSATION_FACTOR': LaunchConfiguration('VEL_COMPENSATION_FACTOR'),
                'ACC_COMPENSATION_FACTOR': LaunchConfiguration('ACC_COMPENSATION_FACTOR'),
                'MIN_TARGET_CONFIDENCE': LaunchConfiguration('MIN_TARGET_CONFIDENCE')  
            }]
        ),
        
        Node(
            package='serial_comman',
            executable='serial_comman_node',
            name='serialcomman',  
            parameters=[{
                'serial': LaunchConfiguration('serial')  
            }],
            output='screen'
        )
    ])