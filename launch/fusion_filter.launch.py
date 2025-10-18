from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fusion_filter_ros2'), 
        'config',
        'fusion_filter.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='fusion_filter_ros2',  
            executable='fusion_filter',  
            name='imu_fusion_filter',
            output='screen',
            parameters=[config],
#            remappings=[('imu', 'imu/data_raw'), ('magnetic_field', 'imu/magnetic_field')]
        ),
    ])
