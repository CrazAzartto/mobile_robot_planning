from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='mpc_controller',
            executable='mpc_node',
            name='mpc_controller',
            output='screen',
            parameters=[{
                'use_sim_time':     use_sim_time,

                # ---- MPC horizon ---
                'horizon_steps':    10,
                'dt':               0.1,

                # ---- Cost weights ---
                'w_goal':           5.0,
                'w_ctrl':           0.1,
                'w_smooth':         1.0,
                'w_terminal':       20.0,
                'w_obstacle':       50.0,

                # ---- Constraints ---
                'v_max':            1.5,
                'omega_max':        2.5,
                'd_safe':           0.4,

                # ---- Obstacle extraction ---
                'obstacle_range':   3.0,
                'max_obstacles':    15,

                # ---- Control ---
                'control_rate':     10.0,
                'goal_tolerance':   0.25,
            }],
        ),
    ])
