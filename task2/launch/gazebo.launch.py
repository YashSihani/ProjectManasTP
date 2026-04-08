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

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': True
        }]
    )

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-z', '0.3',
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

    return LaunchDescription([
        launch_gazebo,
        node_robot_state_publisher,
        delayed_spawn,
    ])
