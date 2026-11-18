from glob import glob
from setuptools import setup

setup(name='omslam_system_monitor', version='0.1.0', packages=['omslam_system_monitor'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/omslam_system_monitor']),
                  ('share/omslam_system_monitor', ['package.xml']),
                  ('share/omslam_system_monitor/launch', glob('launch/*launch.py')),
                  ('share/omslam_system_monitor/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Hyun Lee', maintainer_email='hyunlee@users.noreply.github.com',
      description='Read-only host diagnostics for the robot console', license='Apache-2.0',
      entry_points={'console_scripts': ['system_monitor = omslam_system_monitor.node:main']})
