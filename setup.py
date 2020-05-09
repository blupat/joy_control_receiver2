from setuptools import setup
from glob import glob

package_name = 'joy_control_receiver2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('launch/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blupat',
    maintainer_email='oosmyss@gmail.com',
    description='The joy_control_receiver package with ROS2',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_control_receiver = ' + package_name + '.joy_control_receiver:main',
        ],
    },
)
