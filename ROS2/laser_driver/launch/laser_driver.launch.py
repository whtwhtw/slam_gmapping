from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('model', default_value='X2M'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('frame_id', default_value='laser_frame'),
        Node(
            package='laser_driver',
            executable='laser_driver_node',
            name='laser_driver_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'model': LaunchConfiguration('model'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
