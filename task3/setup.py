from setuptools import setup
import os
from glob import glob

package_name = 'task3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Install Mesh/STL files
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash@todo.todo',
    description='Task 3 for Project Manas Taskphase',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # format: 'executable_name = package_name.file_name:main'
            'lidar_monitor = task3.lidar_monitor:main',
            'pose_tracker = task3.pose_tracker:main',
            'maze_solver = task3.maze_solver:main',
        ],
    },
)
