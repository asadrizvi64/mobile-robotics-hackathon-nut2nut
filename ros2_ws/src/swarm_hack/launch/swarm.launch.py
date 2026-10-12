import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('swarm_hack'), 'config', 'swarm.yaml')
    cfg = LaunchConfiguration('config')
    sim = LaunchConfiguration('sim')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('sim', default_value='false', description='fake drones'),
        DeclareLaunchArgument('drones', default_value='true'),
        DeclareLaunchArgument('mapper', default_value='true'),
        Node(package='swarm_hack', executable='crazyflie_bridge', name='crazyflie_bridge',
             parameters=[cfg, {'sim': sim}], output='screen',
             condition=IfCondition(LaunchConfiguration('drones'))),
        Node(package='swarm_hack', executable='swarm_coordinator', name='swarm_coordinator',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('drones'))),
        Node(package='swarm_hack', executable='object_mapper', name='object_mapper',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('mapper'))),
    ])
