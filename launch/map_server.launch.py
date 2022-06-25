import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_dir=os.path.join(
            get_package_share_directory('ergoCub_ros2'),
            'maps',
            'map.yaml')
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[
                {'yaml_filename': map_dir},
                {'use_sim_time': True}]
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
        )
    ])