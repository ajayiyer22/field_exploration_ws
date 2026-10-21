"""Spawn every robot of config/robots.yaml at the world's spawn poses.

Each robot runs in its own `ros2 launch` process: the Clearpath generators
finish asynchronously, and in a shared launch context a later robot's include
arguments overwrite an earlier robot's. Spawns are staggered.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import FindExecutable, LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from search_common import conda_python, robots, world_config  # noqa: E402


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    stagger = float(LaunchConfiguration('stagger_s').perform(context))
    config = world_config(world)
    actions = []
    for index, robot in enumerate(robots(LaunchConfiguration('robots').perform(context))):
        pose = config['spawn'][robot['namespace']]
        spawn = ExecuteProcess(
            cmd=[conda_python(), FindExecutable(name='ros2'), 'launch',
                 os.path.join(os.path.dirname(__file__), 'spawn_robot.launch.py'),
                 f'platform:={robot["platform"]}', f'namespace:={robot["namespace"]}',
                 f'world:={config["gazebo_world_name"]}',
                 f'x:={pose["x"]}', f'y:={pose["y"]}', f'z:={pose["z"]}', f'yaw:={pose["yaw"]}'],
            output='screen',
        )
        actions.append(TimerAction(period=index * stagger, actions=[spawn]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='hotel'),
        DeclareLaunchArgument('robots', default_value='',
                              description='comma-separated subset, empty = all'),
        DeclareLaunchArgument('stagger_s', default_value='10.0'),
        OpaqueFunction(function=launch_setup),
    ])
