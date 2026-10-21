"""Spawn one Clearpath robot with clearpath_gz's robot_spawn.launch.py.

The Clearpath generators write the robot description and launch files into the
setup directory, so robots/<platform>/robot.yaml is copied to
~/.ros/field_exploration_ws/<namespace>/ first.

    ros2 launch <share>/launch/include/spawn_robot.launch.py platform:=husky_a200 \
        namespace:=a200_0000 x:=11.09 y:=-25.98
"""

import os
import sys

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from search_common import SHARE, setup_dir  # noqa: E402


def launch_setup(context, *args, **kwargs):
    platform = LaunchConfiguration('platform').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    target = setup_dir(namespace)
    os.makedirs(target, exist_ok=True)
    # A platform template can serve multiple robots. Clearpath reads the
    # namespace from robot.yaml, not this launch argument, so specialize the
    # generated instance before running its description and sensor generators.
    with open(os.path.join(SHARE, 'robots', platform, 'robot.yaml')) as stream:
        config = yaml.safe_load(stream)
    config.setdefault('system', {}).setdefault('ros2', {})['namespace'] = namespace
    old_serial = config['serial_number']
    instance = namespace.rsplit('_', 1)[-1]
    if instance.isdigit():
        config['serial_number'] = f'{old_serial.split("-", 1)[0]}-{instance}'
        for host in config['system'].get('hosts', []):
            if host.get('hostname') == f'cpr-{old_serial}':
                host['hostname'] = f'cpr-{config["serial_number"]}'
    with open(os.path.join(target, 'robot.yaml'), 'w') as stream:
        yaml.safe_dump(config, stream, sort_keys=False)

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('clearpath_gz'), 'launch', 'robot_spawn.launch.py')),
        launch_arguments={
            'setup_path': target + '/',
            'world': LaunchConfiguration('world').perform(context),
            'use_sim_time': 'true',
            'rviz': 'false',
            'frame_prefix_relay': 'false',
            'generate': 'true',
            'x': LaunchConfiguration('x').perform(context),
            'y': LaunchConfiguration('y').perform(context),
            'z': LaunchConfiguration('z').perform(context),
            'yaw': LaunchConfiguration('yaw').perform(context),
        }.items(),
    )
    return [spawn]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('platform', default_value='husky_a200'),
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        DeclareLaunchArgument('world', default_value='sim_world',
                              description='Gazebo world name (not the RMF map name)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])
