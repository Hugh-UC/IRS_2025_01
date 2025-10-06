from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pick_nav_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/map', glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group_1',
    maintainer_email='u3276400@uni.canberra.edu.au',
    description='Robot Pick and Place Navigation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hs_waypoint_follower = pick_nav_place.hs_waypoint_follower:main',
            'hs_pick_place = pick_nav_place.hs_pick_place:main',
            'plc_hmi_listener = pick_nav_place.plc_hmi_listener:main',
            'warehouse_coordinator = pick_nav_place.warehouse_coordinator:main',
        ],
    },
)
