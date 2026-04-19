from setuptools import setup
import os
from glob import glob

package_name = 'rl_planner'

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
        (os.path.join('share', package_name, 'models'),
            glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sunny Kumar',
    maintainer_email='student@university.edu',
    description='Reinforcement Learning local planner using Stable Baselines3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rl_node = rl_planner.rl_node:main',
        ],
    },
)
