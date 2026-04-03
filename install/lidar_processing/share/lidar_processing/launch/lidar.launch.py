from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='lidar_processing',
            executable='lidar_filter_node',
            name='lidar_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time':        use_sim_time,
                'min_range':           0.10,
                'max_range':          30.0,
                'median_kernel_size':     5,
                'obstacle_threshold':  0.95,
                'cluster_eps':         0.20,
                'cluster_min_pts':        3,
            }],
            remappings=[('/scan', '/scan')]
        ),
    ])
