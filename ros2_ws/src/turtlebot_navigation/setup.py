from setuptools import find_packages, setup

package_name = 'turtlebot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_robot_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthonyubuntu',
    maintainer_email='anthony.moundalak@net.usj.edu.lb',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver = turtlebot_navigation.robot_driver:main',
            'wall_finder_service = turtlebot_navigation.wall_finder_service:main',
            'lap_time_action_server = turtlebot_navigation.lap_time_action_server:main',
            'lap_time_action_client = turtlebot_navigation.lap_time_action_client:main',
        ],
    },
)
