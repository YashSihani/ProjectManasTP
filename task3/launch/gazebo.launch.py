import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'task3'
    pkg_share = get_package_share_directory(package_name)
    
    # Paths
    robot_urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    robot_description_config = xacro.process_file(robot_urdf_path)
    maze_urdf_path = os.path.join(pkg_share, 'urdf', 'maze.urdf')
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    # 1. Gazebo World
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    # 2. Maze Spawner
    spawn_maze = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', maze_urdf_path, '-entity', 'maze_world', '-x', '16', '-y', '6.0', '-z', '1'],
        parameters=[
                {'use_sim_time': True}],
        output='screen'
    )

    # 3. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': True}]
    )

    # 4. Robot Spawner
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot', '-x', '0.5', '-y', '0.5', '-z', '0.2'],
        output='screen'
    )

    # 5. Task Nodes 
    lidar_monitor_node = Node(
        package=package_name,
        executable='lidar_monitor',
        output='screen'
    )

    pose_tracker_node = Node(
        package=package_name,
        executable='pose_tracker',
        output='screen'
    )
    
    lidar_maze_solver = Node(
        package=package_name,
        executable='maze_solver',
        output='screen'
    )

    return LaunchDescription([
        launch_gazebo,
        TimerAction(period=2.0, actions=[spawn_maze]),
        TimerAction(period=3.0, actions=[node_robot_state_publisher]),
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=7.0, actions=[lidar_monitor_node, pose_tracker_node]),
        TimerAction(period=8.0, actions=[lidar_maze_solver]),

    ])
