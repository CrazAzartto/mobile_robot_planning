from setuptools import setup
import os
from glob import glob

package_name = 'camera_processing'

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
    maintainer='Sunny Kumar',
    maintainer_email='student@university.edu',
    description='Camera preprocessing: rectification, Gaussian blur, HSV segmentation, bounding-box detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = camera_processing.camera_node:main',
        ],
    },
)
