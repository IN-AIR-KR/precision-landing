import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'precision_land_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARK Electronics',
    maintainer_email='info@arkelectron.com',
    description='Python precision landing system using ArUco markers and PX4 OFFBOARD control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tracker = precision_land_py.aruco_tracker_node:main',
            'precision_land = precision_land_py.precision_land_node:main',
            'precision_land_viz = precision_land_py.precision_land_viz_node:main',
        ],
    },
)
