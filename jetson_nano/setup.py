from setuptools import find_packages, setup

package_name = 'rb3_nano_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # Change these
    maintainer_email='your_email@example.com', # Change these
    description='Package for RB3 to Nano communication', # Change if desired
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format is: 'executable_name = package_name.module_name:main_function'
            'rb3_pub = rb3_nano_comm.rb3_publisher:main',
            'nano_sub = rb3_nano_comm.nano_subscriber:main',
        ],
    },
)
