from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'erp42_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='A package for ERP42 obstacle avoidance using LiDAR.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = erp42_navigation.lidar_sensor_node:main',
            'obstacle_avoider = erp42_navigation.obstacle_avoidance_node:main',
            'vehicle_driver = erp42_navigation.vehicle_driver_node:main',
        ],
    },
)
