from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ergoCub_ros2',
            executable='ProtoCode',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])