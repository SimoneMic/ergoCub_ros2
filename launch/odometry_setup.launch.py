import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    chest_projector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/chest_projection.launch.py'])
        )

    return LaunchDescription([
        chest_projector,

        Node(
            package='ergoCub_ros2',
            executable='odometry_standalone',
            output='screen',
            parameters=[{'use_sim_time': 'true'}]
        )
    ])