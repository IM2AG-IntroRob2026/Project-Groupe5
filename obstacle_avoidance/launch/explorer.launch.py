from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace, for example Robot5',
        ),
        Node(
            package='obstacle_avoidance',
            executable='explorer',
            name='explorer',
            namespace=namespace,
            output='screen',
        ),
    ])
