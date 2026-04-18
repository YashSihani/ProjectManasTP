import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'task2'
    pkg_share = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    robot_description_config = xacro.process_file(xacro_file)

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': True
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.3',
            '-Y', '0.032029530880000004',
        ],
        output='screen'
    )

    lidar_monitor_node = Node(
        package='task2',
        executable='lidar_monitor',
        output='screen'
    )

    pose_tracker_node = Node(
        package='task2',
        executable='pose_tracker',
        output='screen'
    )

    action_server_node = Node(
        package='task2',
        executable='action_server',
        output='screen'
    )

    return LaunchDescription([
        # 0s — Gazebo starts
        launch_gazebo,

        # 3s — state publisher starts
        TimerAction(period=3.0, actions=[
            node_robot_state_publisher,
        ]),

        # 5s — robot spawns
        TimerAction(period=5.0, actions=[
            spawn_entity,
        ]),

        # 8s — sensors and pose tracker start
        TimerAction(period=7.0, actions=[
            lidar_monitor_node,
            pose_tracker_node,
        ]),

        # 10s — action server starts
        TimerAction(period=9.0, actions=[
            action_server_node,
        ]),
    ])
