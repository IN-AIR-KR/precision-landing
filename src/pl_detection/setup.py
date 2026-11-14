from setuptools import setup
setup(name='pl_detection', version='0.0.1', packages=['pl_detection'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pl_detection']),
                  ('share/pl_detection', ['package.xml'])],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='Precision Landing Maintainers', maintainer_email='jake@arkelectron.com',
      description='Python ArUco detector with common ROS camera inputs', license='BSD-3-Clause',
      entry_points={'console_scripts': ['aruco_tracker = pl_detection.aruco_tracker:main']})
