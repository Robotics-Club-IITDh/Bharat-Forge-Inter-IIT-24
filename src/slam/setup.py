from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/controller_robot_1.yaml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),  # Added comma here
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*'))  # Removed extra comma
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cobaltboy',
    maintainer_email='pathanaffan62@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'operator_node = slam.operatorNode:main',
            'nearest_bot = slam.nearest_bot_node:main'
        ],
    },
)