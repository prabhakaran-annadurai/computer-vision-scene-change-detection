from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='/path/to/my_custom_world.world',
            description='Path to the world file'
        ),
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen',
        ),
    ])

