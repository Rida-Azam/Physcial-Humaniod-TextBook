from setuptools import setup

package_name = 'simple_perception_examples'

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
    description='Simple perception examples for humanoid robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_object_detector = simple_perception_examples.simple_object_detector:main',
            'simple_image_subscriber = simple_perception_examples.simple_image_subscriber:main',
        ],
    },
)