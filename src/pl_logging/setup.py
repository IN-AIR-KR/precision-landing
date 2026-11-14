from setuptools import setup

setup(
    name='pl_logging',
    version='0.0.1',
    packages=['pl_logging'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pl_logging']),
        ('share/pl_logging', ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Precision Landing Maintainers',
    maintainer_email='jake@arkelectron.com',
    description='Precision landing rosbag recorder',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'flight_recorder = pl_logging.flight_recorder:main',
    ]},
)
