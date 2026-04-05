import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rov_cameras'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hydromeda',
    maintainer_email='hydromeda@todo.todo',
    description='ROV onboard nodes — cameras, joystick translation, control',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'photogrammetry_node = rov_cameras.photogrammetry_node:main',
            'joy_to_mavlink = rov_cameras.joy_to_mavlink:main',
            'arm_service = rov_cameras.arm_service:main',
            'mavlink_bridge = rov_cameras.mavlink_bridge:main',
        ],
    },
)
