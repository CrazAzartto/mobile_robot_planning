from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='apf_planner',
            executable='apf_node',
            name='apf_planner',
            output='screen',
            parameters=[{
                'use_sim_time':          use_sim_time,

                # ---- Attractive (conic-well) ---
                'k_att':                 1.2,
                'd_star':                1.5,

                # ---- Repulsive (inverse-square) ---
                'k_rep':                 1.5,
                'd_influence':           1.0,
                'd_safe':                0.25,

                # ---- Velocity limits ---
                'max_linear_vel':        3.5,
                'max_angular_vel':       4.0,
                'force_to_vel_scale':    1.5,

                # ---- Goal ---
                'goal_tolerance':        0.20,

                # ---- Local minima escape ---
                'stuck_vel_thresh':      0.03,
                'stuck_time_thresh':     3.0,
                'escape_rotate_time':    1.5,
                'escape_move_time':      3.0,
                'escape_angular_vel':    1.5,
                'escape_linear_vel':     0.5,

                # ---- Control rate ---
                'control_rate':         30.0,
            }],
        ),
    ])
