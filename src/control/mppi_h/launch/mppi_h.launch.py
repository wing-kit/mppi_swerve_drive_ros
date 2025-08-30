from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mppi_h')
    params_file = os.path.join(pkg_share, 'config', 'mppi_h.yaml')

    return LaunchDescription([
        Node(
            package='mppi_h',
            executable='mppi_h_node',
            name='mppi_h',
            output='screen',
            parameters=[params_file]
        )
    ])

