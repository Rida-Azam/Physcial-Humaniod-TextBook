from setuptools import setup, find_packages

package_name = 'isaac_sim_integration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include launch files
        ('share/' + package_name + '/launch', [
            'launch/integrated_simulation.launch.py',
        ]),

        # Include config files
        ('share/' + package_name + '/config', [
            'config/integration_config.yaml',
        ]),

        # Include any other necessary files
        ('share/' + package_name + '/scripts', [
            'scripts/setup_simulation_env.sh',
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'transforms3d',
        'opencv-python',
        'pyquaternion',
        'torch',
        'torchvision',
        'transformers',
        'openai-whisper',
    ],
    zip_safe=True,
    maintainer='Maintainer Name',
    maintainer_email='maintainer@todo.todo',
    description='Isaac Sim integration package with physics, rendering, and sensor simulation for humanoid robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_sim_integration_node = isaac_sim_integration.main:main',
        ],
    },
)