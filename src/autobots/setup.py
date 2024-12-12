from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'autobots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/autobot_launch.py']),
        (os.path.join('share', package_name, 'launch'), ['launch/singlebot.py']),
        (os.path.join('share', package_name, 'launch'), ['launch/multi_robot_launch.py']),
        (os.path.join('share', package_name, 'launch'), ['launch/robot2_launch.py']),
        (os.path.join('share', package_name, 'launch'), ['launch/robot3_launch.py']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/model.urdf.xacro']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Doe',
    maintainer_email='johndoe@sample.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'waypoint_follower = autobots.waypoint_follower:main',
        'waypoint_follower_robot2 = autobots.waypoint_follower_robot2:main',
        'waypoint_follower_robot3 = autobots.waypoint_follower_robot3:main',
        ],
    },
)
