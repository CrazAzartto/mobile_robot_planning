from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='evaluation',
            executable='eval_node',
            name='eval_node',
            output='screen',
            parameters=[{
                'use_sim_time':          use_sim_time,
                'output_dir':            'eval_results',
                'collision_threshold':   0.25,
                'near_miss_threshold':   0.5,
                'goal_tolerance':        0.25,
                'publish_rate':          2.0,
            }],
        ),
    ])
