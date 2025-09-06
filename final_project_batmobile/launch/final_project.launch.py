from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_project_batmobile',
            executable='cmd_arbiter',
            name='cmd_arbiter',
            output='screen'
        ),
        Node(
            package='final_project_batmobile',
            executable='constant_speed',
            name='constant_speed',
            output='screen'
        ),
        Node(
            package='final_project_batmobile',
            executable='stopsign_executable',
            name='stopsign_detector_node',
            output='screen'
        ),
    ])
