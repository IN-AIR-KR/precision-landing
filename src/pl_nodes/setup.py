import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pl_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@local',
    description='Precision landing detector and controller nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v_marker_detector = pl_nodes.v_marker_detector_node:main',
            'aruco_detector = pl_nodes.aruco_detector_node:main',
            'landing_controller = pl_nodes.landing_controller_node:main',
        ],
    },
)
