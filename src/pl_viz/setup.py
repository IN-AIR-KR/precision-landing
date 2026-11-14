from setuptools import setup

setup(
    name='pl_viz',
    version='0.0.1',
    packages=['pl_viz'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pl_viz']),
        ('share/pl_viz', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Jacob Dahl',
    maintainer_email='jake@arkelectron.com',
    description='Common precision landing camera and telemetry monitor',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['monitor = pl_viz.monitor:main']},
)
