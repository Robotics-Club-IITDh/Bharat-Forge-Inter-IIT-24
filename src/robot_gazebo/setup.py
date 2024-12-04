from setuptools import find_packages, setup

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/world_launch.py']),
        ('share/' + package_name + '/world', ['world/empty.world']),
        ('share/' + package_name + '/world', ['world/factory.world']),
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
            'map_merger = robot_gazebo.map_merger:main',
            'master_controller = robot_gazebo.master_controller:main'
        ],
    },
)
