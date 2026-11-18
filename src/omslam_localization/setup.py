from glob import glob
from setuptools import setup

setup(
    name='omslam_localization', version='0.1.0', packages=['omslam_localization'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/omslam_localization']),
                ('share/omslam_localization', ['package.xml', 'README.md']),
                ('share/omslam_localization/launch', glob('launch/*.launch.py')),
                ('share/omslam_localization/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Oh My SLAM maintainers', maintainer_email='maintainers@example.com',
    description='Wheel encoder and external IMU EKF odometry', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'imu_conditioner = omslam_localization.imu_conditioner:main',
        'hfi_a9_node = omslam_localization.hfi_a9_node:main',
        'mock_imu = omslam_localization.mock_imu:main',
    ]},
)
