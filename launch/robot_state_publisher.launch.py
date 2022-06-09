import imp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_urdf = "$(command 'cat /home/user1/robotology-superbuild/build/install/share/iCub/robots/stickBot/model.urdf')"

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_urdf]
    )

    return LaunchDescription(robot_state_publisher_node)

