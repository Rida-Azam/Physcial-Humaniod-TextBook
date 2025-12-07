from setuptools import setup

package_name = 'demo_nodes_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Example ROS 2 packages in Python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = demo_nodes_py.publisher_member_function:main',
            'listener = demo_nodes_py.subscriber_member_function:main',
            'add_two_ints_server = demo_nodes_py.add_two_ints_server:main',
            'add_two_ints_client = demo_nodes_py.add_two_ints_client:main',
            'fibonacci_action_server = demo_nodes_py.fibonacci_action_server:main',
            'fibonacci_action_client = demo_nodes_py.fibonacci_action_client:main',
        ],
    },
)