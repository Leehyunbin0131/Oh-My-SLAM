from setuptools import setup
from glob import glob
setup(name='omslam_base', version='0.1.0', packages=['omslam_base'],
      data_files=[('share/ament_index/resource_index/packages',['resource/omslam_base']),
                  ('share/omslam_base',['package.xml']),('share/omslam_base/launch',glob('launch/*launch.py')),
                  ('share/omslam_base/config',glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Hyun Lee', maintainer_email='hyunlee@users.noreply.github.com',
      description='Lifecycle wheel bridge and measured odometry', license='Apache-2.0',
      entry_points={'console_scripts':['base_bridge = omslam_base.node:main']})
