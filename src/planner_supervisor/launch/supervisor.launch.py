from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    augmentation_mode = LaunchConfiguration('augmentation_mode', default='mpc')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('augmentation_mode', default_value='mpc',
                              description='Augmentation: mpc, rl, or none'),

        Node(
            package='planner_supervisor',
            executable='supervisor_node',
            name='planner_supervisor',
            output='screen',
            parameters=[{
                'use_sim_time':          use_sim_time,
                'augmentation_mode':     augmentation_mode,
                'control_rate':          30.0,
                'goal_tolerance':        0.25,

                # Stuck detection
                'stuck_vel_thresh':      0.05,
                'stuck_time_thresh':     3.0,
                'stuck_progress_thresh': 0.1,

                # Augmentation
                'augment_timeout':       15.0,
                'augment_progress':      0.5,

                # Safety
                'safety_dist':           0.20,
            }],
        ),
    ])
