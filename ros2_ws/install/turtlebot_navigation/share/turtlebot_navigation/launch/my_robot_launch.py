from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_navigation',
            executable='robot_driver',
            name='robot_driver',
            output='screen'
        ),
        Node(
            package='turtlebot_navigation',
            executable='wall_finder_service',
            name='wall_finder_service',
            output='screen'
        ),
        Node(
            package='turtlebot_navigation',
            executable='lap_time_action_server',
            name='lap_time_action_server',
            output='screen'
        ),
        Node(
            package='turtlebot_navigation',
            executable='lap_time_action_client',
            name='lap_time_action_client',
            output='screen'
        ),
    ])

