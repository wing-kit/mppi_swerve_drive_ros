from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mppi_4d')
    params_file = os.path.join(pkg_share, 'config', 'mppi_4d.yaml')

    return LaunchDescription([
        Node(
            package='mppi_4d',
            executable='mppi_4d_node',
            name='mppi_4d',
            output='screen',
            parameters=[params_file]
        )
    ])

