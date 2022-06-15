import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

package_name = 'ergoCub_ros2'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_dir=os.path.join(
            get_package_share_directory(package_name),
            'maps',
            'map.yaml')
    
    param_file_name = 'ergoCub_amcl_nav2' + '.yaml'

    param_dir=os.path.join(
            get_package_share_directory(package_name),
            'param',
            param_file_name)
            
    amcl_params = os.path.join(os.getcwd(), param_dir)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            emulate_tty=True,
            parameters=[{'yaml_filename': map_dir},
                        {'topic_name': 'map'},
                        {'frame_id': 'map'}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='map_server_lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[amcl_params]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['amcl']}]
        )
    ])