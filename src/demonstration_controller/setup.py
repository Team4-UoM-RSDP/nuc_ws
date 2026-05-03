from setuptools import find_packages, setup

package_name = 'demonstration_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student18',
    maintainer_email='stasim49@gmail.com',
    description='TODO: Package description',
    license='BSD-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detect_and_pick.py = demonstration_controller.object_detect_and_pick.py:main',
            'object_detect_pick_place.py = demonstration_controller.object_detect_pick_place.py:main'
        ],
    },
)
