from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
           package='vrx_sub',
            namespace='mission_planner',
            executable='mission_planner',
            name='mission_planner'
        ),
        Node(
           package='vrx_sub',
            namespace='path_planner',
            executable='path_planner',
            name='path_planner',
        ),
        Node(
           package='vrx_sub',
            namespace='control',
            executable='control',
            name='control'
        ),
        Node(
           package='vrx_sub',
            namespace='robot_loc',
            executable='robot_loc',
            name='robot_loc'
        )
    ])
