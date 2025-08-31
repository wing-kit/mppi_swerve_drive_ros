from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('joy_controller')
    joy_params = os.path.join(pkg_share, 'config', 'joy.yaml')
    controller_params = os.path.join(pkg_share, 'config', 'joy_controller.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_params]
        ),
        Node(
            package='joy_controller',
            executable='joy_controller_node',
            name='joy_controller',
            output='screen',
            parameters=[controller_params]
        )
    ])

