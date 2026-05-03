from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'leo_joy_example'

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages',
      ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='pi',
  maintainer_email='pi@todo.todo',
  description='TODO: Package description',
  license='TODO: License declaration',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': ["manager_node = ""leo_joy_example.manager_node:main",
                        'object_detect_and_pick = demonstration_controller.object_detect_and_pick:main',
    ],
  },
)