"""
full_system.launch.py
=====================
Master launch file — starts the complete pipeline:

  1. Gazebo simulation  (robot_simulation)
  2. LiDAR filter node  (lidar_processing)
  3. Camera node        (camera_processing)
  4. Sensor fusion node (sensor_fusion)
  5. APF planner        (apf_planner)

Usage:
  ros2 launch robot_simulation full_system.launch.py
  ros2 launch robot_simulation full_system.launch.py goal_x:=12.0 goal_y:=0.0
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
    pkg_sim     = get_package_share_directory('robot_simulation')
    pkg_lidar   = get_package_share_directory('lidar_processing')
    pkg_camera  = get_package_share_directory('camera_processing')
    pkg_fusion  = get_package_share_directory('sensor_fusion')
    pkg_apf     = get_package_share_directory('apf_planner')

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    start_rviz_arg   = DeclareLaunchArgument(
        'start_rviz', default_value='true')
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='8.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz   = LaunchConfiguration('start_rviz')
    goal_x       = LaunchConfiguration('goal_x')
    goal_y       = LaunchConfiguration('goal_y')

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
    # 4. Sensor fusion                                                     #
    # ------------------------------------------------------------------ #
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fusion, 'launch', 'fusion.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 5. APF planner                                                       #
    # ------------------------------------------------------------------ #
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_apf, 'launch', 'planner.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------------------------------------------ #
    # 6. Goal publisher — sends a single PoseStamped to /goal_pose        #
    #    after all nodes are running                                       #
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
    # Assemble with delays so Gazebo is fully loaded first                #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        goal_x_arg,
        goal_y_arg,

        LogInfo(msg='[full_system] Starting Gazebo simulation...'),
        simulation,

        # Wait for Gazebo + robot spawn, then start processing nodes
        TimerAction(period=6.0, actions=[
            LogInfo(msg='[full_system] Starting sensor processing nodes...'),
            lidar_launch,
            camera_launch,
        ]),

        # Fusion needs both sensors active first
        TimerAction(period=8.0, actions=[
            LogInfo(msg='[full_system] Starting sensor fusion...'),
            fusion_launch,
        ]),

        # Planner starts last — needs odom, scan_filtered, fused_obstacles
        TimerAction(period=10.0, actions=[
            LogInfo(msg='[full_system] Starting APF planner...'),
            planner_launch,
            goal_publisher,
        ]),
    ])
