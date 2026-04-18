from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task2'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages() handles the nested task2/task2/ directory
    packages=[package_name],
    
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash22.mitmpl2025@learner.manipal.edu',
    description='Task 2 Robot Description and Launch',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The format is: 'executable_name = folder_name.file_name:function'
            'lidar_monitor = task2.lidar_monitor:main',
            'action_client = task2.action_client:main',
            'pose_tracker = task2.pose_tracker:main',
            'action_server = task2.action_server:main',
        ],
    },
)
