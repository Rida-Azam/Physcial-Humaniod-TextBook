from setuptools import setup
from glob import glob
import os

package_name = 'perception_pipeline'

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
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Model files
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        # Scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Perception pipeline for humanoid robots including VSLAM and object detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vslam_node = perception_pipeline.vslam_node:main',
            'object_detection_node = perception_pipeline.object_detection_node:main',
            'perception_fusion_node = perception_pipeline.perception_fusion_node:main',
        ],
    },
)