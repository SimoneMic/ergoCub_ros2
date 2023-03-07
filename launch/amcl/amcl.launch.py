import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

package_name = 'ergoCub_ros2'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
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