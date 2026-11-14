from glob import glob

from setuptools import setup

setup(
    name='pl_camera',
    version='0.0.1',
    packages=['pl_camera'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pl_camera']),
        ('share/pl_camera', ['package.xml']),
        ('share/pl_camera/launch', glob('launch/*.launch.py')),
        ('share/pl_camera/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Precision Landing Maintainers',
    maintainer_email='jake@arkelectron.com',
    description='Camera inputs for precision landing',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'csi_camera = pl_camera.csi_camera:main',
            'image_relay = pl_camera.image_relay:main',
        ],
    },
)
