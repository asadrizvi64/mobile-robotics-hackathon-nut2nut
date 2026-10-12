"""SLAM or localization + Nav2 + nav_manager for a TurtleBot (ROS 2 Humble).

  ros2 launch tb_nav tb_nav.launch.py mode:=slam
  ros2 launch tb_nav tb_nav.launch.py mode:=localization map:=$HOME/arena_map.yaml

Nav2 params default to the TurtleBot3 ones shipped in turtlebot3_navigation2
(tuned footprint, speeds, AMCL). Override with params_file:=...
"""
import glob
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def find_nav2_params(model):
    """Nav2's own Humble params. (The installed TB3 params use Jazzy-style plugin
    names that Humble's planner_server rejects.)"""
    return os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')


def _unused_find_tb3_params(model):
    try:
        share = get_package_share_directory('turtlebot3_navigation2')
        hits = glob.glob(os.path.join(share, 'param', '**', f'{model}.yaml'), recursive=True)
        if hits:
            hits.sort(key=lambda p: ('humble' not in p, len(p)))
            return hits[0]
    except PackageNotFoundError:
        pass
    return os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')


def setup(context):
    cfg = context.launch_configurations
    mode = cfg['mode']
    params = cfg['params_file'] or find_nav2_params(cfg['model'])
    sim_time = cfg['use_sim_time']
    nav2 = get_package_share_directory('nav2_bringup')
    actions = [LogInfo(msg=f'[tb_nav] mode={mode}  nav2 params={params}')]

    if mode == 'slam' and cfg['run_slam'] != 'true':
        actions.append(LogInfo(msg='[tb_nav] run_slam:=false -> expecting slam_toolbox on the robot'))
    elif mode == 'slam':
        slam_share = get_package_share_directory('slam_toolbox')
        slam_args = {'use_sim_time': sim_time,
                     'slam_params_file': cfg['slam_params_file'] or os.path.join(
                         get_package_share_directory('tb_nav'), 'config', 'slam_tb3.yaml')}
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments=slam_args.items()))
    elif mode == 'localization':
        map_yaml = os.path.expanduser(cfg['map'])
        if not os.path.isfile(map_yaml):
            raise RuntimeError(f'[tb_nav] map file not found: {map_yaml}  (pass map:=/full/path/map.yaml)')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'localization_launch.py')),
            launch_arguments={'map': map_yaml, 'use_sim_time': sim_time,
                              'params_file': params, 'autostart': 'true'}.items()))
    else:
        raise RuntimeError(f'[tb_nav] mode must be slam or localization, got {mode}')

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': sim_time, 'params_file': params, 'autostart': 'true'}.items()))

    actions.append(Node(
        package='tb_nav', executable='nav_manager', name='nav_manager', output='screen',
        parameters=[os.path.join(get_package_share_directory('tb_nav'), 'config', 'nav_manager.yaml'),
                    {'mode': mode, 'use_sim_time': sim_time == 'true'}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam', description='slam | localization'),
        DeclareLaunchArgument('map', default_value='', description='map yaml (localization mode)'),
        DeclareLaunchArgument('model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'burger')),
        DeclareLaunchArgument('params_file', default_value='', description='Nav2 params override'),
        DeclareLaunchArgument('slam_params_file', default_value=''),
        DeclareLaunchArgument('run_slam', default_value='true',
                              description='false = slam_toolbox runs on the robot instead'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
