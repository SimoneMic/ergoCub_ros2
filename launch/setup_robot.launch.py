import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/robot_state_publisher.launch.py'])
        )
    scan_filtering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/scan_filtering.launch.py'])
        )
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/ergoCub_rviz.launch.py'])
        )

    return LaunchDescription([
        state_publisher,
        scan_filtering,
        rviz_node,
    ])