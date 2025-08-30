from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mppi_3d')
    params_file = os.path.join(pkg_share, 'config', 'mppi_3d_a.yaml')

    return LaunchDescription([
        Node(
            package='mppi_3d',
            executable='mppi_3d_node',
            name='mppi_3d',
            output='screen',
            parameters=[params_file]
        )
    ])

