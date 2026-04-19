"""
full_system.launch.py
=====================
Master launch file — starts the complete pipeline:

  1. Gazebo simulation  (robot_simulation)
  2. LiDAR filter node  (lidar_processing)
  3. Camera node        (camera_processing)
  4. Sensor fusion node (sensor_fusion) — with Kalman tracking
  5. APF planner        (apf_planner)   — velocity-aware repulsive field
  6. MPC controller     (mpc_controller) — receding-horizon local planner
  7. RL planner         (rl_planner)     — learned policy (fallback reactive)
  8. Planner supervisor (planner_supervisor) — APF/MPC/RL switching
  9. Evaluation node    (evaluation)     — metrics logging

Usage:
  ros2 launch robot_simulation full_system.launch.py
  ros2 launch robot_simulation full_system.launch.py goal_x:=12.0 goal_y:=0.0
  ros2 launch robot_simulation full_system.launch.py planner_mode:=rl
  ros2 launch robot_simulation full_system.launch.py planner_mode:=none  # pure APF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Package share directories                                            #
    # ------------------------------------------------------------------ #
    pkg_sim        = get_package_share_directory('robot_simulation')
    pkg_lidar      = get_package_share_directory('lidar_processing')
    pkg_camera     = get_package_share_directory('camera_processing')
    pkg_fusion     = get_package_share_directory('sensor_fusion')
    pkg_apf        = get_package_share_directory('apf_planner')
    pkg_mpc        = get_package_share_directory('mpc_controller')
    pkg_rl         = get_package_share_directory('rl_planner')
    pkg_supervisor = get_package_share_directory('planner_supervisor')
    pkg_eval       = get_package_share_directory('evaluation')

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    start_rviz_arg   = DeclareLaunchArgument(
        'start_rviz', default_value='true')
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='12.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='5.0')
    planner_mode_arg = DeclareLaunchArgument(
        'planner_mode', default_value='mpc',
        description='Augmentation mode: mpc, rl, or none (pure APF)')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    start_rviz     = LaunchConfiguration('start_rviz')
    goal_x         = LaunchConfiguration('goal_x')
    goal_y         = LaunchConfiguration('goal_y')
    planner_mode   = LaunchConfiguration('planner_mode')

    # ------------------------------------------------------------------ #
    # 1. Simulation                                                        #
    # ------------------------------------------------------------------ #
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_rviz':   start_rviz,
        }.items()
    )

    # ------------------------------------------------------------------ #
    # 2. LiDAR processing  (delay 5 s to allow Gazebo + robot to start)  #
    # ------------------------------------------------------------------ #
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'lidar.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 3. Camera processing                                                 #
    # ------------------------------------------------------------------ #
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_camera, 'launch', 'camera.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 4. Sensor fusion (with Kalman tracker)                               #
    # ------------------------------------------------------------------ #
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fusion, 'launch', 'fusion.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 5. APF planner (supervised mode — publishes to /apf_cmd_vel)        #
    # ------------------------------------------------------------------ #
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_apf, 'launch', 'planner.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 6. MPC controller                                                    #
    # ------------------------------------------------------------------ #
    mpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mpc, 'launch', 'mpc.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 7. RL planner                                                        #
    # ------------------------------------------------------------------ #
    rl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rl, 'launch', 'rl.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 8. Planner supervisor                                                #
    # ------------------------------------------------------------------ #
    supervisor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_supervisor, 'launch', 'supervisor.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'augmentation_mode': planner_mode,
        }.items()
    )

    # ------------------------------------------------------------------ #
    # 9. Evaluation node                                                   #
    # ------------------------------------------------------------------ #
    eval_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_eval, 'launch', 'eval.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 10. Goal publisher — sends a single PoseStamped to /goal_pose       #
    #     after all nodes are running                                      #
    # ------------------------------------------------------------------ #
    goal_publisher = Node(
        package='robot_simulation',
        executable='goal_publisher.py',
        name='goal_publisher',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'use_sim_time': use_sim_time,
        }]
    )

    # ------------------------------------------------------------------ #
    # 11. Pedestrian mover — moves dynamic obstacle models in Gazebo     #
    # ------------------------------------------------------------------ #
    dynamic_mover = Node(
        package='robot_simulation',
        executable='dynamic_mover.py',
        name='dynamic_mover',
        output='screen',
        parameters=[{
            'world_name': 'obstacle_world',
            'update_rate': 10.0,
            'use_sim_time': use_sim_time,
        }]
    )

    # ------------------------------------------------------------------ #
    # Assemble with delays so Gazebo is fully loaded first                #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        goal_x_arg,
        goal_y_arg,
        planner_mode_arg,

        LogInfo(msg='[full_system] Starting Gazebo simulation...'),
        simulation,

        # Wait for Gazebo + robot spawn, then start processing nodes
        TimerAction(period=13.0, actions=[
            LogInfo(msg='[full_system] Starting sensor processing nodes...'),
            lidar_launch,
            camera_launch,
            LogInfo(msg='[full_system] Starting dynamic mover (pedestrians + cars)...'),
            dynamic_mover,
        ]),

        # Fusion needs both sensors active first
        TimerAction(period=15.0, actions=[
            LogInfo(msg='[full_system] Starting sensor fusion (with Kalman tracker)...'),
            fusion_launch,
        ]),

        # Planners + supervisor + evaluation start last
        TimerAction(period=17.0, actions=[
            LogInfo(msg='[full_system] Starting APF planner + MPC + RL + Supervisor...'),
            planner_launch,
            mpc_launch,
            rl_launch,
            supervisor_launch,
            eval_launch,
            goal_publisher,
        ]),
    ])
