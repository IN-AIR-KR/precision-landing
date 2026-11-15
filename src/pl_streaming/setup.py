from setuptools import setup

setup(
    name='pl_streaming',
    version='0.0.1',
    packages=['pl_streaming'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pl_streaming']),
        ('share/pl_streaming', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Precision Landing Maintainers',
    maintainer_email='jake@arkelectron.com',
    description='Low-latency live HUD streaming to MediaMTX',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['hud_streamer = pl_streaming.hud_streamer:main']},
)
