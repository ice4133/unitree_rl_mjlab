from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    task = LaunchConfiguration('task').perform(context)
    if task not in ('perception', 'mapping', 'navigation'):
        raise RuntimeError('task must be perception, mapping or navigation')
    actions = []
    for package, file, parameters in [
        ('autonomy_perception', 'perception.launch.py', ('lidar_params', 'scan_params')),
        *([('autonomy_localization', 'mapping.launch.py', ('slam_params',))]
          if task in ('mapping', 'navigation') else []),
        *([('autonomy_navigation', 'navigation.launch.py', ('nav_params',))]
          if task == 'navigation' else []),
    ]:
        file_path = Path(get_package_share_directory(package)) / 'launch' / file
        arguments = {'use_sim_time': LaunchConfiguration('use_sim_time')}
        arguments.update({name: LaunchConfiguration(name) for name in parameters})
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(file_path)),
                                                launch_arguments=arguments.items()))
    rviz = Path(get_package_share_directory('autonomy_bringup')) / 'rviz' / (
        'navigation.rviz' if task == 'navigation' else 'mapping.rviz')
    actions.append(Node(package='rviz2', executable='rviz2', arguments=[
        '-d', str(rviz), '-f', 'odom' if task == 'perception' else 'map'],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
        output='screen'))
    return actions


def generate_launch_description():
    perception = Path(get_package_share_directory('autonomy_perception'))
    localization = Path(get_package_share_directory('autonomy_localization'))
    navigation = Path(get_package_share_directory('autonomy_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='mapping', description='perception, mapping, or online SLAM + navigation'),
        DeclareLaunchArgument('ros_domain_id', default_value='10', description='ROS domain, independent of SDK2 domain 0'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use the simulator /clock'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Open task visualization'),
        DeclareLaunchArgument('lidar_params', default_value=str(perception / 'config/lidar.yaml')),
        DeclareLaunchArgument('scan_params', default_value=str(perception / 'config/scan.yaml')),
        DeclareLaunchArgument('slam_params', default_value=str(localization / 'config/slam.yaml')),
        DeclareLaunchArgument('nav_params', default_value=str(navigation / 'config/nav2.yaml')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        OpaqueFunction(function=start),
    ])
