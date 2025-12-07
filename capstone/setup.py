from setuptools import setup
from glob import glob
import os

package_name = 'capstone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # RViz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Capstone project for autonomous humanoid from spoken command',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_processor = capstone.voice_processor:main',
            'nlu_processor = capstone.nlu_processor:main',
            'task_planner = capstone.task_planner:main',
            'action_executor = capstone.action_executor:main',
            'safety_monitor = capstone.safety_monitor:main',
            'robot_interface = capstone.robot_interface:main',
        ],
    },
)