"""Nav2 (Smac 2D planner + MPPI controller) for one robot namespace.

Every server runs in /<namespace>, reads TF from /<namespace>/tf and publishes
TwistStamped on /<namespace>/cmd_vel (Clearpath's twist_mux input).
"""

import math
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

sys.path.insert(0, os.path.dirname(__file__))
from search_common import SHARE, footprint, robots  # noqa: E402


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot = robots(namespace)[0]
    template = os.path.join(SHARE, 'config', robot['nav2_params'])
    # Cover the padded rectangle in every orientation, with two extra 5 cm
    # cells. MPPI must see nonzero centre costs whenever the footprint could
    # touch an obstacle; an inscribed-radius-only layer does not provide that.
    padded_radius = math.hypot(0.5 * float(robot['length']) + 0.05,
                               0.5 * float(robot['width']) + 0.05)
    local_inflation_radius = math.ceil((padded_radius + 0.10) / 0.05) * 0.05
    params = ParameterFile(
        RewrittenYaml(
            source_file=ReplaceString(
                source_file=template,
                replacements={
                    '<robot>': namespace,
                    '<footprint>': f'"{footprint(robot)}"',
                    '<vx_max>': str(float(robot['v_max'])),
                    '<wz_max>': str(float(robot['w_max'])),
                    '<local_inflation_radius>': str(local_inflation_radius),
                    # Half the width: closest a wall can be to the robot centre.
                    '<inscribed_radius>': str(0.5 * min(float(robot['length']),
                                                        float(robot['width'])) + 0.01),
                },
            ),
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    motion = remappings + [('cmd_vel', 'cmd_vel'), ('/trajectories', 'mppi_trajectories')]
    bt_xml = os.path.join(SHARE, 'config', 'navigate_to_pose_exploration.xml')
    lifecycle_nodes = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']

    def server(package, executable, extra_remaps=None, overrides=None):
        return Node(
            package=package, executable=executable, name=executable, namespace=namespace,
            parameters=[params, overrides or {}],
            remappings=extra_remaps if extra_remaps is not None else remappings,
            output='screen')

    return [
        server('nav2_planner', 'planner_server'),
        server('nav2_controller', 'controller_server', motion),
        server('nav2_behaviors', 'behavior_server', motion),
        server('nav2_bt_navigator', 'bt_navigator',
               overrides={'default_nav_to_pose_bt_xml': bt_xml}),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace=namespace,
            parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 30.0,
                         'node_names': lifecycle_nodes}],
            output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        OpaqueFunction(function=launch_setup),
    ])
