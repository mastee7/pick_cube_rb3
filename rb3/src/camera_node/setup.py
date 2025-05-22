from setuptools import setup
import os
from glob import glob

package_name = 'camera_node' #'my_camera_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If you have launch files, add them here
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='je', # CHANGE ME
    maintainer_email='jel166@ucsd.edu', # CHANGE ME
    description='ROS 2 node to publish and save camera images.', # CHANGE ME
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package_name.module_name:main_function'
            'image_publisher_saver = camera_node.image_publisher_node:main',
        ],
    },
)
