from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='scanner', 
            default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='ergoCub_ros2',
            executable='scan_filter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/filtered_pc2'),
                        ('scan', '/filtered_scan')],
            parameters=[{
                'target_frame': 'lidar',
                'transform_tolerance': 0.01,
                'min_height': -300.0,  #-300
                'max_height': 300.0,  #300
                'angle_min': -3.141592653,  # -M_PI
                'angle_max': 3.141592653,  # M_PI
                'angle_increment': 0.003926991,  # 2M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 25.0,
                'use_inf': True,
                'inf_epsilon': 1.0
                #'concurrency_level': 2
            }],
            name='pointcloud_to_laserscan'
        )
    ])
    