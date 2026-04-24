import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    gmapping_dir = get_package_share_directory('gmapping')
    rsp_pkg_dir = get_package_share_directory('robot_state_publisher_pkg')
    urdf_path = os.path.join(rsp_pkg_dir, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_desc}]),
        Node(package='odometry_publisher', executable='odometry_publisher',
             name='odometry_publisher', output='screen',
             parameters=[{'odom_frame': 'odom', 'base_frame': 'base_link', 'publish_rate': 30.0}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gmapping_dir, 'launch', 'slam_gmapping.launch.py')),
            launch_arguments={'scan_topic': '/scan', 'base_frame': 'base_link',
                              'odom_frame': 'odom', 'map_frame': 'map'}.items()),
    ])
