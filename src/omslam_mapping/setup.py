from glob import glob
from setuptools import setup

setup(
    name='omslam_mapping', version='0.1.0', packages=['omslam_mapping'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/omslam_mapping']),
        ('share/omslam_mapping', ['package.xml', 'README.md']),
        ('share/omslam_mapping/launch', glob('launch/*.launch.py')),
        ('share/omslam_mapping/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Oh My SLAM maintainers', maintainer_email='maintainers@example.com',
    description='2D SLAM and saved-map localization', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'save_map = omslam_mapping.map_bundle:save_main',
        'load_graph = omslam_mapping.map_bundle:load_main',
    ]},
)
