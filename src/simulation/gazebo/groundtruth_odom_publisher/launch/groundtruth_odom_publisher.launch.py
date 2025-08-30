from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('groundtruth_odom_publisher')
    params_file = os.path.join(pkg_share, 'config', 'groundtruth_odom_publisher.yaml')

    return LaunchDescription([
        Node(
            package='groundtruth_odom_publisher',
            executable='groundtruth_odom_publisher_node',
            name='groundtruth_odom_publisher',
            output='screen',
            parameters=[params_file]
        )
    ])

