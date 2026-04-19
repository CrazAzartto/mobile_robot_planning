from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_path = LaunchConfiguration('model_path', default='')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='Path to trained SB3 model (.zip)'),

        Node(
            package='rl_planner',
            executable='rl_node',
            name='rl_planner',
            output='screen',
            parameters=[{
                'use_sim_time':     use_sim_time,
                'model_path':       model_path,
                'v_max':            1.5,
                'omega_max':        2.5,
                'control_rate':     10.0,
                'goal_tolerance':   0.25,
                'n_lidar_sectors':  8,
                'lidar_range':      5.0,
            }],
        ),
    ])
