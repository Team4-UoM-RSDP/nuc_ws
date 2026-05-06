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
            'object_detect_and_pick = demonstration_controller.object_detect_and_pick:main',
            'look_then_pick_just_manipulator = demonstration_controller.look_then_pick_just_manipulator:main',
            'move_to_block_pick = demonstration_controller.move_to_block_pick:main',
            'object_detect_pick_place = demonstration_controller.object_detect_pick_place:main',
            'demo_takeover = demonstration_controller.demo_takeover:main',
            'testing_script = demonstration_controller.testing_script:main',
            'initial_pos_test = demonstration_controller.initial_pos_test:main'

        ],
    },
)
