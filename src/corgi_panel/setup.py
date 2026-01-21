#!/usr/bin/env python3
"""
Setup script for corgi_panel ROS 2 Python package
"""
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'corgi_panel'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']) + ['scripts'],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files (if any exist)
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        
        # Install assets (QSS stylesheets, images, etc.)
        (os.path.join('share', package_name, 'assets'),
            glob('corgi_ui/assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anne',
    maintainer_email='r13522820@ntu.edu.tw',
    description='Corgi Robot Control and Configuration Panels with PyQt5 GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'corgi_control_panel = scripts.run_control:main',
            'corgi_config_panel = scripts.run_config:main',
        ],
    },
)
