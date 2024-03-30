from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')  
    
    file_path = f'{package_dir}/config/config.yaml'

    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)

    return LaunchDescription([
    ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        cwd=config['path']['micro_agent'],
        shell=True
    ),
    ExecuteProcess(
        cmd=['make', 'px4_sitl', 'jmavsim'],
        cwd=config['path']['px4_sitl'],
        shell=True
    ),
    Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='visualizer',
        name='visualizer'
    ),
    Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='offboard_control',
        name='control',
        parameters=[{'radius': 10.0}, {'altitude': 6.0}, {'omega': 0.5}]
    ),
    Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
    ),
    Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='takeoff_offboard',
        name='takeoff'
    ),
])