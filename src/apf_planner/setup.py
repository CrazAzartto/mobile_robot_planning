from setuptools import setup
import os
from glob import glob

package_name = 'apf_planner'

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
    description='Artificial Potential Field path planner with conic-well attractive and inverse-square repulsive potentials',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apf_node = apf_planner.apf_node:main',
        ],
    },
)
