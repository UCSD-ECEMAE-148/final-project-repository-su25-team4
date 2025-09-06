from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_project_batmobile',
            executable='cmd_arbiter_color',
            name='cmd_arbiter_color',
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
            executable='color_detector',
            name='color_detection_final',
            output='screen'
        ),
    ])

