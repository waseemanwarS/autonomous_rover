#!/usr/bin/env python3

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rov_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files - THIS IS THE CRITICAL PART
        (os.path.join('share', package_name, 'launch'),
            glob('rov_bringup/launch/*.launch.py')),
        
        # Include any config files if you have them
        # (os.path.join('share', package_name, 'config'),
        #     glob('rov_bringup/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Rover bringup package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)