import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/map_server.launch.py'])
        )
    scan_filtering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/ergoCub_amcl_nav2.launch.py'])
        )

    return LaunchDescription([
        map_server,
        scan_filtering
    ])