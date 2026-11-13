import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('eyerov_motor_sim'),
        'config', 'motor_sim.yaml')
    return LaunchDescription([
        Node(
            package='eyerov_motor_sim',
            executable='motor_sim_node',
            name='motor_sim_node',
            output='screen',
            parameters=[config],
        ),
    ])
