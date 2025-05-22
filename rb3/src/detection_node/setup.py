from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'detection_node'

# Function to recursively collect files from a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

# Collect all files from the resource directory
resource_files = package_files(os.path.join(package_name, 'share/resource'))
# If output dir is relative within package source, you might choose NOT to install it.
# output_files = package_files(os.path.join(package_name, 'detection_node/output'))


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']), # Automatically find packages like 'detection_node'
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files if any (example)
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # --- IMPORTANT: Install resource files ---
        # Installs files from 'share/resource' into '<install_dir>/share/detection_node/resource'
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('share/resource', '*.*'))), # Install files directly in resource
        (os.path.join('share', package_name, 'resource/data'), glob(os.path.join('share/resource/data', '*.*'))), # Install files in resource/data

        # --- DO NOT Install the output directory ---
        # Typically, output generated at runtime shouldn't be part of installation data_files
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'], # Add numpy if not already implied
    zip_safe=True,
    maintainer='your_name', # CHANGE ME
    maintainer_email='your_email@example.com', # CHANGE ME
    description='ROS 2 node for YOLO object detection using OpenCV DNN.', # CHANGE ME
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             # Format: 'executable_name = package_name.module_name:main_function'
            'yolo_subscriber = detection_node.detection_subscriber_node:main',
        ],
    },
)
