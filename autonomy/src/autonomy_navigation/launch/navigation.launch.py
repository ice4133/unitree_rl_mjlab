"""Humble Nav2 with a single, monitored output to the Go2W auto receiver."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('autonomy_navigation'))
    parameters = [LaunchConfiguration('nav_params'), {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'collision_monitor'),
    ]
    nodes = []
    for package, executable in servers:
        remappings = []
        if executable in ('controller_server', 'behavior_server', 'velocity_smoother'):
            remappings.append(('cmd_vel', '/navigation/cmd_vel_raw'))
        if executable == 'velocity_smoother':
            remappings.append(('cmd_vel_smoothed', '/navigation/cmd_vel_smoothed'))
        nodes.append(Node(package=package, executable=executable, name=executable,
                          parameters=parameters, remappings=remappings, output='screen'))
    nodes.append(Node(
        package='autonomy_navigation', executable='velocity_guard', name='velocity_guard',
        parameters=parameters, output='screen'))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                     'autostart': True, 'node_names': [name for _, name in servers]}]))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav_params', default_value=str(share / 'config/nav2.yaml')),
        *nodes,
    ])
