import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_simulation  = get_package_share_directory('robot_simulation')
    pkg_ros_gz_sim        = get_package_share_directory('ros_gz_sim')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    start_rviz_arg   = DeclareLaunchArgument('start_rviz', default_value='true')

    robot_x_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    robot_y_arg = DeclareLaunchArgument('y_pose', default_value='-1.5')
    robot_z_arg = DeclareLaunchArgument('z_pose', default_value='0.2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz   = LaunchConfiguration('start_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robot_description'),
            'urdf', 'robot.urdf.xacro'
        ])
    ])

    robot_description = {
    	'robot_description': ParameterValue(robot_description_content, value_type=str)
     }

    world_file = os.path.join(pkg_robot_simulation, 'worlds', 'obstacle_world.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file]
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mobile_robot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_robot_description, 'rviz', 'robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz)
    )

    # ros_gz_bridge node for standard messages
    bridge_params = os.path.join(pkg_robot_simulation, 'config', 'bridge.yaml')
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # ros_gz_image node specifically for camera images
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,

        gazebo,
        robot_state_publisher,
        TimerAction(period=8.0, actions=[spawn_robot]),
        static_tf,
        TimerAction(period=10.0, actions=[rviz]),
        gazebo_bridge,
        image_bridge,
    ])
