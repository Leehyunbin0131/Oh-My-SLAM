from glob import glob
from setuptools import setup

setup(
    name='omslam_navigation', version='0.1.0', packages=['omslam_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/omslam_navigation']),
        ('share/omslam_navigation', ['package.xml', 'LICENSE']),
        ('share/omslam_navigation/launch', glob('launch/*.launch.py')),
        ('share/omslam_navigation/config', glob('config/*.yaml')),
        ('share/omslam_navigation/rviz', glob('rviz/*.rviz')),
        ('share/omslam_navigation/config/lattice', glob('config/lattice/*')),
        ('share/omslam_navigation/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Hyun Lee', maintainer_email='hyunlee@users.noreply.github.com',
    description='State Lattice and DWB simulation navigation', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'navigation_guard = omslam_navigation.navigation_guard:main',
    ]},
)
