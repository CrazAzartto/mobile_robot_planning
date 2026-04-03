from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='camera_processing',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'use_sim_time':    use_sim_time,
                'gaussian_kernel': 5,
                'gaussian_sigma':  1.5,
                'min_bbox_area':   300,
                'max_bbox_area':   50000,
                'publish_debug':   True,
            }],
            remappings=[
                ('/camera/image_raw',   '/camera/image_raw'),
                ('/camera/camera_info', '/camera/camera_info'),
            ]
        ),
    ])
