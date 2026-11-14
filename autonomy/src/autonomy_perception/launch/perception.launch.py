from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('autonomy_perception'))
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lidar_params', default_value=str(share / 'config/lidar.yaml')),
        DeclareLaunchArgument('scan_params', default_value=str(share / 'config/scan.yaml')),
        Node(package='autonomy_perception', executable='lidar_filter_node', name='lidar_filter',
             parameters=[LaunchConfiguration('lidar_params'), sim_time], output='screen'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', parameters=[LaunchConfiguration('scan_params'), sim_time],
             remappings=[('cloud_in', '/perception/lidar/points_filtered'), ('scan', '/scan')],
             output='screen'),
    ])
