from glob import glob
from setuptools import setup
setup(name='omslam_operator', version='0.1.0', packages=['omslam_operator'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/omslam_operator']),
                  ('share/omslam_operator', ['package.xml', 'LICENSE']),
                  ('share/omslam_operator/qml', glob('qml/*.qml')),
                  ('share/omslam_operator/qml/design', glob('qml/design/*.qml')),
                  ('share/omslam_operator/config', glob('config/*')),
                  ('share/omslam_operator/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Hyun Lee', maintainer_email='hyunlee@users.noreply.github.com',
      description='Native Qt robot operator console', license='Apache-2.0',
      tests_require=['pytest'], entry_points={'console_scripts': [
          'console = omslam_operator.bootstrap:main']})
