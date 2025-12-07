from setuptools import setup
from glob import glob
import os

package_name = 'isaac_gym_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Model files
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        # Scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Reinforcement Learning walking policies for humanoid robots using Isaac Gym',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_walking_policy = isaac_gym_rl.rl_walking_policy:main',
            'train_walking_policy = isaac_gym_rl.train_walking_policy:main',
        ],
    },
)