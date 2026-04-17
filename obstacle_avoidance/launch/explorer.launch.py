from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='Robot5'),
        DeclareLaunchArgument('linear_speed', default_value='0.2'),
        DeclareLaunchArgument('angular_speed', default_value='0.45'),
        DeclareLaunchArgument('exploration_time', default_value='60.0'),

        # Node 1: Autonomous Explorer
        Node(
            package='obstacle_avoidance',
            executable='explorer',
            namespace=ns,
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'exploration_time': LaunchConfiguration('exploration_time')
            }]
        ),

        # Node 2: Teleop Keyboard Handler (Opening in a separate xterm window)
        Node(
            package='obstacle_avoidance',
            executable='teleop',
            namespace=ns,
            prefix='xterm -e', # This launches a new terminal window for focus
            output='screen'
        )
    ])