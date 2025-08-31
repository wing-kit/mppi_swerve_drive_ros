from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('vel_driver')
    params_file = os.path.join(pkg_share, 'config', 'vel_driver.yaml')

    return LaunchDescription([
        Node(
            package='vel_driver',
            executable='vel_driver_node',
            name='vel_driver',
            output='screen',
            parameters=[params_file]
        )
    ])

