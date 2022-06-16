from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    world = '/usr/share/gazebo-11/worlds/SmallWarehouseScen3.world'

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_yarp_clock.so'],
            output='screen')
    #GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    #spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
    #                    arguments=['-entity', 'demo', 'x', 'y', 'z'],
    #                    output='screen')
    #spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                    arguments=['-entity', 'stickBot', '0', '0.0', '1.0'],
    #                    output='screen')
    return LaunchDescription([
        gazebo,
        #spawn_entity,
    ])