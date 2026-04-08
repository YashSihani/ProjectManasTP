import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'task2'
    
    # 1. Locate the URDF file inside the 'install' directory
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    # 2. Read the actual text content of the URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. Create the Robot State Publisher node
    # This node takes the URDF and converts it into a ROS topic (/robot_description)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. Create the Joint State Publisher node
    # Required for RViz to calculate the positions of your robot's parts
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # 5. Create the RViz2 node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 6. Tell ROS to launch all three at once
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz
    ])
