from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'deconfliction_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package metadata
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akash',
    maintainer_email='akash@todo.todo',
    description='Drone mission deconfliction engine with simulation and 4D trajectory support',
    license='MIT',  # or your preferred license
    tests_require=['pytest'],
    extras_require={
        'launch': ['launch'],
        'dependencies': ['rclpy', 'std_msgs', 'geometry_msgs', 'visualization_msgs', 'deconfliction_msgs'],

    },
    entry_points={
        'console_scripts': [
            'mission_input_node = deconfliction_system.mission_input_node:main',
            'simulation_manager = deconfliction_system.simulation_manager:main',
            'deconfliction_engine = deconfliction_system.deconfliction_engine:main',
            'visualization_node = deconfliction_system.visualization_node:main',
        ],
    },
)
