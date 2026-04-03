from setuptools import setup
import os
from glob import glob

package_name = 'lidar_processing'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sunil Bishnoi',
    maintainer_email='student@university.edu',
    description='LiDAR data filtering and obstacle extraction node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_filter_node = lidar_processing.lidar_filter_node:main',
        ],
    },
)
