"""Start an rmf_demos world in Gazebo Harmonic and keep its doors in place.

The world comes from rmf_demos_maps; search_bringup only adds the sensor
systems Clearpath robots need (prepared at build time) and launches it through
rmf_demos_gz's simulation.launch.xml.

    ros2 launch search_bringup sim.launch.py world:=hotel headless:=false
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from search_common import conda_python, world_config  # noqa: E402


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    config = world_config(world)
    rmf_map = config['rmf_map_name']

    rmf_maps = get_package_share_directory('rmf_demos_maps')
    clearpath_gz = get_package_share_directory('clearpath_gz')
    # Models of the RMF world, Clearpath meshes, and every sourced package so
    # package:// URIs in the robot descriptions resolve.
    shares = [os.path.join(p, 'share') for p in os.environ['AMENT_PREFIX_PATH'].split(':')]
    resource_path = ':'.join(
        [os.path.join(rmf_maps, 'maps', rmf_map, 'models'),
         os.path.join(clearpath_gz, 'worlds'),
         os.path.join(clearpath_gz, 'meshes')] + shares +
        [p for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':') if p])

    rmf_share = get_package_share_directory('rmf_demos_gz')
    simulation_file = os.path.join(rmf_share, 'launch', 'simulation.launch.xml')
    if not os.path.isfile(simulation_file):
        # The hotel-exploration fork installs XML launch files at the share root.
        simulation_file = os.path.join(rmf_share, 'simulation.launch.xml')

    simulation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(simulation_file),
        launch_arguments={
            'map_package': 'search_bringup',
            'map_name': rmf_map,
            'world_path': os.path.join(get_package_share_directory('search_bringup'),
                                       'maps', rmf_map, rmf_map + '.world'),
            'headless': headless,
        }.items(),
    )
    doors = config.get('doors', {})
    door_keeper = Node(
        package='search_sim', executable='door_keeper', name='door_keeper',
        prefix=[conda_python()],
        parameters=[{
            'use_sim_time': True,
            'open_doors': doors.get('open', []) or [''],
            'closed_doors': doors.get('closed', []) or [''],
        }],
        output='screen',
    )
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
            simulation, door_keeper]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='hotel'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
