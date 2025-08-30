from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('map_visualizer')
    params_file = os.path.join(pkg_share, 'config', 'map_visualizer.yaml')

    map_name_arg = DeclareLaunchArgument('map_name', default_value='')

    return LaunchDescription([
        map_name_arg,
        Node(
            package='map_visualizer',
            executable='map_visualizer_node',
            name='map_visualizer',
            output='screen',
            parameters=[params_file, {'map_name': LaunchConfiguration('map_name')}]
        )
    ])

