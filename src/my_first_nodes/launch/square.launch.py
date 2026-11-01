from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    side_length_arg = DeclareLaunchArgument(
        'side_length', default_value='2.0', description='正方形边长（米）')

    return LaunchDescription([
        side_length_arg,

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),
        Node(
            package='my_first_nodes',
            executable='draw_square',
            name='draw_square',
            output='screen',
            parameters=[{
                'side_length': LaunchConfiguration('side_length'),
                'linear_speed': 1.0,
                'angular_speed': 0.5,
            }],
        ),
    ])