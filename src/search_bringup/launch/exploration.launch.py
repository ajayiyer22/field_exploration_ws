"""Exploration task: WFD frontier detection, Hungarian allocation, the
exploration dispatcher (Nav2 goal execution) and mission coordination.

    ros2 launch search_bringup exploration.launch.py

Run it after simulation.launch.py. The mission coordinator waits until every
robot's pose, the map and Nav2 are ready, then starts exploring. Parameters are
in config/exploration.yaml; the team is config/robots.yaml.
"""

import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'include'))
from search_common import SHARE, robots  # noqa: E402


def generate_launch_description():
    team = robots()
    namespaces = [r['namespace'] for r in team]
    params = os.path.join(SHARE, 'config', 'exploration.yaml')
    goal_max_cost = [float(r['goal_max_cost']) for r in team]

    def node(executable, name, extra):
        return Node(
            package='search_exploration', executable=executable, name=name,
            parameters=[params, {'robot_namespaces': namespaces, **extra}],
            output='screen')

    return LaunchDescription([
        node('frontier_detector_node', 'frontier_detector', {
            'goal_max_cost': goal_max_cost,
        }),
        node('task_allocator_node', 'task_allocator', {
            'robot_goal_prefixes': [r['goal_label_prefix'] for r in team],
            'goal_max_cost': goal_max_cost,
        }),
        node('exploration_dispatcher_node', 'exploration_dispatcher', {
            'goal_max_cost': goal_max_cost,
        }),
        node('mission_coordinator_node', 'mission_coordinator', {}),
    ])
