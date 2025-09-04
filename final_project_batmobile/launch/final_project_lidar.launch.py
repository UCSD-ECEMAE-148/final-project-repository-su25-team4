
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_project_batmobile',
            executable='cmd_arbiter_lidar',
            name='cmd_arbiter_lidar',
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
            executable='lidar_detection_final',
            name='lidar_dectection_final',
            output='screen'
        ),
    ])



