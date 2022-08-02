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
    #deprecated
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
    odom_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/odometry_setup.launch.py'])
        )
    odom_gt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/odometry_gt.launch.py'])
        )
    projection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/chest_projection.launch.py'])
        )
    scan_filtering_compensated = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/scan_filtering_compensated.launch.py'])
        )
    depth_to_pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ergoCub_ros2'), 'launch'),
            '/point_cloud_xyz.launch.py'])
        )

    return LaunchDescription([
        state_publisher,
        projection_node,
        rviz_node,
        scan_filtering_compensated,
        odom_node,
        depth_to_pointcloud
    ])