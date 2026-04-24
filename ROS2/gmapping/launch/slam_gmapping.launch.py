import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    gmapping_dir = get_package_share_directory('gmapping')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    map_frame = LaunchConfiguration('map_frame', default='map')

    # GMapping parameters
    map_update_interval = LaunchConfiguration('map_update_interval', default='5.0')
    max_urange = LaunchConfiguration('max_urange', default='16.0')
    sigma = LaunchConfiguration('sigma', default='0.05')
    kernel_size = LaunchConfiguration('kernel_size', default='1')
    lstep = LaunchConfiguration('lstep', default='0.05')
    astep = LaunchConfiguration('astep', default='0.05')
    iterations = LaunchConfiguration('iterations', default='5')
    lsigma = LaunchConfiguration('lsigma', default='0.075')
    ogain = LaunchConfiguration('ogain', default='3.0')
    lskip = LaunchConfiguration('lskip', default='0')
    srr = LaunchConfiguration('srr', default='0.1')
    srt = LaunchConfiguration('srt', default='0.2')
    str_ = LaunchConfiguration('str', default='0.1')
    stt = LaunchConfiguration('stt', default='0.2')
    linear_update = LaunchConfiguration('linear_update', default='1.0')
    angular_update = LaunchConfiguration('angular_update', default='0.5')
    temporal_update = LaunchConfiguration('temporal_update', default='3.0')
    resample_threshold = LaunchConfiguration('resample_threshold', default='0.5')
    particles = LaunchConfiguration('particles', default='30')
    xmin = LaunchConfiguration('xmin', default='-50.0')
    ymin = LaunchConfiguration('ymin', default='-50.0')
    xmax = LaunchConfiguration('xmax', default='50.0')
    ymax = LaunchConfiguration('ymax', default='50.0')
    delta = LaunchConfiguration('delta', default='0.05')
    llsamplerange = LaunchConfiguration('llsamplerange', default='0.01')
    llsamplestep = LaunchConfiguration('llsamplestep', default='0.01')
    lasamplerange = LaunchConfiguration('lasamplerange', default='0.005')
    lasamplestep = LaunchConfiguration('lasamplestep', default='0.005')

    # Define the slam_gmapping node
    slam_gmapping_node = Node(
        package='gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'throttle_scans': 1,
            'base_frame': base_frame,
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'map_update_interval': map_update_interval,
            'maxUrange': max_urange,
            'sigma': sigma,
            'kernelSize': kernel_size,
            'lstep': lstep,
            'astep': astep,
            'iterations': iterations,
            'lsigma': lsigma,
            'ogain': ogain,
            'lskip': lskip,
            'srr': srr,
            'srt': srt,
            'str': str_,
            'stt': stt,
            'linearUpdate': linear_update,
            'angularUpdate': angular_update,
            'temporalUpdate': temporal_update,
            'resampleThreshold': resample_threshold,
            'particles': particles,
            'xmin': xmin,
            'ymin': ymin,
            'xmax': xmax,
            'ymax': ymax,
            'delta': delta,
            'llsamplerange': llsamplerange,
            'llsamplestep': llsamplestep,
            'lasamplerange': lasamplerange,
            'lasamplestep': lasamplestep,
        }],
        remappings=[
            ('scan', scan_topic),
        ],
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('scan_topic', default_value='scan',
                              description='Topic name of laser scan data'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                              description='Odometry frame'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='Map frame'),

        # GMapping parameters
        DeclareLaunchArgument('map_update_interval', default_value='5.0',
                              description='Map update interval in seconds'),
        DeclareLaunchArgument('max_urange', default_value='16.0',
                              description='Maximum usable range of the laser'),
        DeclareLaunchArgument('sigma', default_value='0.05',
                              description='Sigma of the scan matching'),
        DeclareLaunchArgument('kernel_size', default_value='1',
                              description='Kernel size for scan matching'),
        DeclareLaunchArgument('lstep', default_value='0.05',
                              description='Linear search step'),
        DeclareLaunchArgument('astep', default_value='0.05',
                              description='Angular search step'),
        DeclareLaunchArgument('iterations', default_value='5',
                              description='Number of iterations for scan matching'),
        DeclareLaunchArgument('lsigma', default_value='0.075',
                              description='Likelihood sigma for scan matching'),
        DeclareLaunchArgument('ogain', default_value='3.0',
                              description='Likelihood gain for scan matching'),
        DeclareLaunchArgument('lskip', default_value='0',
                              description='Number of beams to skip'),
        DeclareLaunchArgument('srr', default_value='0.1',
                              description='Linear noise in linear motion'),
        DeclareLaunchArgument('srt', default_value='0.2',
                              description='Angular noise in linear motion'),
        DeclareLaunchArgument('str', default_value='0.1',
                              description='Linear noise in angular motion'),
        DeclareLaunchArgument('stt', default_value='0.2',
                              description='Angular noise in angular motion'),
        DeclareLaunchArgument('linear_update', default_value='1.0',
                              description='Trigger update after moving this distance'),
        DeclareLaunchArgument('angular_update', default_value='0.5',
                              description='Trigger update after rotating this angle'),
        DeclareLaunchArgument('temporal_update', default_value='3.0',
                              description='Trigger update after this time'),
        DeclareLaunchArgument('resample_threshold', default_value='0.5',
                              description='Resample threshold'),
        DeclareLaunchArgument('particles', default_value='30',
                              description='Number of particles'),
        DeclareLaunchArgument('xmin', default_value='-50.0',
                              description='Minimum x map size'),
        DeclareLaunchArgument('ymin', default_value='-50.0',
                              description='Minimum y map size'),
        DeclareLaunchArgument('xmax', default_value='50.0',
                              description='Maximum x map size'),
        DeclareLaunchArgument('ymax', default_value='50.0',
                              description='Maximum y map size'),
        DeclareLaunchArgument('delta', default_value='0.05',
                              description='Map resolution'),
        DeclareLaunchArgument('llsamplerange', default_value='0.01',
                              description='Likelihood sampler range'),
        DeclareLaunchArgument('llsamplestep', default_value='0.01',
                              description='Likelihood sample step'),
        DeclareLaunchArgument('lasamplerange', default_value='0.005',
                              description='Angular sampler range'),
        DeclareLaunchArgument('lasamplestep', default_value='0.005',
                              description='Angular sample step'),

        # SLAM node
        slam_gmapping_node,
    ])
