from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'vehicle'),
            glob('models/vehicle/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aduff001',
    maintainer_email='aduff001@terpmail.umd.edu',
    description='Solution to UMD Loop 2026 Challenge Week Phase 1 problem S1',
    license='GNU GPLv3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_scanner = waypoint_nav.lidar_scanner:main',
            'random_cube_spawner = waypoint_nav.random_cube_spawner:main',
            'vehicle_controller = waypoint_nav.vehicle_controller:main',
            'waypoint_generator = waypoint_nav.waypoint_generator:main',
        ],
    },
)
