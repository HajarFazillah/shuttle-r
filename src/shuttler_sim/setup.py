import os
from glob import glob
from setuptools import setup

package_name = 'shuttler_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hajarf',
    maintainer_email='hajarf@todo.todo',
    description='Shuttler simulation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shuttlecock_detector = shuttler_sim.shuttlecock_detector:main',
            'teleop_keyboard = shuttler_sim.teleop_keyboard:main',
            'shuttlecock_seeker = shuttler_sim.shuttlecock_seeker:main',
            'shuttlecock_collector = shuttler_sim.shuttlecock_collector:main',
            'scoop_follower = shuttler_sim.scoop_follower:main',
        ],
    },
)

