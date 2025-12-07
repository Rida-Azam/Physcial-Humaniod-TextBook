from setuptools import setup

package_name = 'isaac_sim_ros_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', ['launch/isaac_sim_bridge.launch.py']),
        # Include config files
        ('share/' + package_name + '/config', ['config/bridge_params.yaml']),
        # Include any other necessary files
        ('share/' + package_name + '/scripts', ['scripts/setup_isaac_sim_env.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer Name',
    maintainer_email='maintainer@todo.todo',
    description='ROS 2 bridge for Isaac Sim humanoid robotics simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_sim_bridge = isaac_sim_ros_bridge.ros_bridge:main',
        ],
    },
)