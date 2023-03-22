"""webots_ros2 package setup file."""

from glob import glob
from setuptools import setup
import os


package_name = 'webots_ros2_p3at'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

#launch files
data_files.append((os.path.join('share', package_name), glob('launch/*.py')))
#proto/world files
data_files.append(('share/' + package_name + '/protos', glob('protos/*.proto')))
data_files.append(('share/' + package_name + '/protos', glob('protos/*.stl')))
data_files.append(('share/' + package_name + '/protos/textures', glob('protos/textures/*')))
data_files.append(('share/' + package_name + '/worlds', glob('worlds/*')))

#maps
data_files.append(('share/' + package_name + '/maps', glob('maps/*')))
#configs
data_files.append(('share/' + package_name + '/config', glob('config/*')))
#params
data_files.append(('share/' + package_name + '/params', glob('params/*')))

data_files.append((os.path.join('share', package_name), glob('config/*')))

setup(
    name=package_name,
    version='1.0.6',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='P3AT robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p3at_driver = webots_ros2_p3at.p3at_driver:main',
            'navfix2can = webots_ros2_p3at.navfix2can:main',
            'gps_renamer = webots_ros2_p3at.gps_renamer:main',
            'detect_blob = webots_ros2_p3at.detect_blob:main',
            'goto = webots_ros2_p3at.goto:main',
            'path_server = webots_ros2_p3at.path_server:main',
        ],
    }
)
