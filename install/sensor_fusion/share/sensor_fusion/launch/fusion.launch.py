from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time':       use_sim_time,
                'sync_slop':          0.10,
                'lidar_frame':        'laser_link',
                'camera_frame':       'camera_optical_link',
                'camera_hfov_deg':    80.0,
                'lidar_min_range':    0.10,
                'lidar_max_range':   30.0,
                'bbox_depth_margin':  0.05,
            }],
        ),
    ])
