from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare Launch Configurations
    namespace = LaunchConfiguration('namespace')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    exploration_time = LaunchConfiguration('exploration_time')

    return LaunchDescription([
        # 2. Declare Arguments (so they can be passed via command line)
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace (e.g., Robot5)'
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.15',
            description='Forward speed in m/s'
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.5',
            description='Turning speed in rad/s'
        ),
        DeclareLaunchArgument(
            'exploration_time',
            default_value='60.0',
            description='Total mission time in seconds'
        ),

        # 3. The Main Explorer Node (Autonomous Logic)
        Node(
            package='obstacle_avoidance',
            executable='explorer',
            name='explorer_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'exploration_time': exploration_time,
            }]
        ),

        # 4. The Keyboard Handler Node (Manual Override)
        Node(
            package='obstacle_avoidance',
            executable='keyboard_handler',
            name='keyboard_handler_node',
            namespace=namespace,
            output='screen',
            # Open in a separate terminal to capture keyboard input
            emulate_tty=True,
            prefix='xterm -e' 
        ),
    ])