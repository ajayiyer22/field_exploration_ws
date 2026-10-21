"""Ground-truth localization and the shared exploration map."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from search_common import SHARE, conda_python, robots, world_config  # noqa: E402


def launch_setup(context, *args, **kwargs):
    config = world_config(LaunchConfiguration('world').perform(context))
    team = robots(LaunchConfiguration('robots').perform(context))
    namespaces = [r['namespace'] for r in team]
    grid = config['grid']
    localizer = Node(
        package='search_sim', executable='ground_truth_localizer',
        name='ground_truth_localizer',
        prefix=[conda_python()],
        parameters=[{
            'use_sim_time': True,
            'world': config['gazebo_world_name'],
            'robot_namespaces': namespaces,
        }],
        output='screen',
    )
    exploration_map = Node(
        package='exploration_map', executable='exploration_map', name='exploration_map',
        prefix=[conda_python()],
        parameters=[
            os.path.join(SHARE, 'config', 'exploration_map.yaml'),
            {
                'use_sim_time': True,
                'robot_namespaces': namespaces,
                'robot_lengths': [float(r['length']) for r in team],
                'robot_widths': [float(r['width']) for r in team],
                'grid_origin_x': float(grid['origin_x']),
                'grid_origin_y': float(grid['origin_y']),
                'grid_size_x_m': float(grid['size_x_m']),
                'grid_size_y_m': float(grid['size_y_m']),
                'grid_resolution': float(grid['resolution']),
            },
        ],
        output='screen',
    )
    costmap_visualizer = Node(
        package='search_sim', executable='costmap_visualizer', name='costmap_visualizer',
        prefix=[conda_python()],
        parameters=[{'use_sim_time': True, 'robot_namespaces': namespaces}],
        output='screen',
    )
    jackal_number = 0
    names = []
    for robot in team:
        if robot['platform'] == 'jackal_j100':
            jackal_number += 1
            names.append(f'Jackal {jackal_number}')
        elif robot['platform'] == 'husky_a200':
            names.append('Husky')
        else:
            names.append(robot['namespace'])
    allocation_display = Node(
        package='search_sim', executable='task_allocation_display',
        name='task_allocation_display', prefix=[conda_python()],
        parameters=[{'use_sim_time': True, 'robot_namespaces': namespaces,
                     'robot_names': names,
                     'robot_goal_prefixes': [r['goal_label_prefix'] for r in team]}], output='screen',
    )
    return [localizer, exploration_map, costmap_visualizer, allocation_display]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='hotel'),
        DeclareLaunchArgument('robots', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
