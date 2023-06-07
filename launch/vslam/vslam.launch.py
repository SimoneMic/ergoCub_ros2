from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    param_dir=os.path.join(
            get_package_share_directory('ergoCub_ros2'),
            'param',
            'stella_vslam.yaml')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    load_nodes = GroupAction(
                    actions=[Node(
                                    package='stella_vslam_ros',
                                    executable='run_slam',
                                    output='screen',
                                    parameters=[configured_params],
                                    ros_arguments=['-v $(find ergoCub_ros2)/vocabulary/orb_vocab.fbow -c $(find ergoCub_ros2)/param/realsense.yaml']
                                )])
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(load_nodes)
    return ld
    #return LaunchDescription([
    #    Node(
    #        package='stella_vslam_ros',
    #        executable='run_slam',
    #        output='screen',
    #        parameters=[configured_params]
    #    )
    #])