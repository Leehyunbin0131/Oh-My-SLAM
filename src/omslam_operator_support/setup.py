from setuptools import setup
setup(name='omslam_operator_support', version='0.1.0', packages=['omslam_operator_support'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/omslam_operator_support']),
                  ('share/omslam_operator_support', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, tests_require=['pytest'],
      entry_points={'console_scripts': ['console_session = omslam_operator_support.session:main']})
