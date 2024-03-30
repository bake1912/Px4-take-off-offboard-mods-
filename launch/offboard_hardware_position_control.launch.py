from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='offboard_control',
            name='control',
            parameters= [{'radius': 2.0},{'altitude': 3.0},{'omega': 0.5}]
        )
    ])
