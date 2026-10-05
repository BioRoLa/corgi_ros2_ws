#!/usr/bin/env python3
"""Setup script for corgi_orin_monitor (ament_python, no ROS runtime dependency)."""
import os
from glob import glob

from setuptools import setup

package_name = 'corgi_orin_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changtsailin',
    maintainer_email='angel.tl314@gmail.com',
    description='Always-on Orin health/power logger and reboot post-mortem report.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orin_monitor = corgi_orin_monitor.monitor:main',
            'orin_monitor_report = corgi_orin_monitor.report:main',
        ],
    },
)
