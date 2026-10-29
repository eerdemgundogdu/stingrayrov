from setuptools import setup
import os
from glob import glob

package_name = 'stingray_mobility'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stingray Team',
    maintainer_email='stingray@team.com',
    description='STINGRAY su altı aracı hareket edebilirlik ROS2 node\'ları',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = stingray_mobility.serial_bridge_node:main',
            'sensor_node = stingray_mobility.sensor_node:main',
            'controller = stingray_mobility.controller_node:main',
            'mission = stingray_mobility.mission_node:main',
            'safety = stingray_mobility.safety_node:main',
            'vision = stingray_mobility.vision_node:main',
        ],
    },
)
