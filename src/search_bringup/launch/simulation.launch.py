"""Simulation: Hotel world, Husky + two Jackals, ground-truth localization,
exploration map and costmaps, Nav2 for every robot, CUDA YOLO26, and RViz.

    ros2 launch search_bringup simulation.launch.py
    ros2 launch search_bringup simulation.launch.py headless:=true rviz:=false

Start the exploration task afterwards with

    ros2 launch search_bringup exploration.launch.py

The team is config/robots.yaml and the world is config/worlds/hotel.yaml.

Run inside the field_robotics Conda environment (Python 3.12, the version ROS
Jazzy is built for), after sourcing ROS and this workspace. ~/.local
site-packages are ignored so they cannot shadow the Conda packages, and Gazebo
transport is kept on its own partition, away from other sessions.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

INCLUDE = os.path.join(os.path.dirname(__file__), 'include')
sys.path.insert(0, INCLUDE)
from search_common import SHARE, robots  # noqa: E402

WORLD = 'hotel'
SPAWN_DELAY_S = 8.0       # Gazebo up before the first robot
SPAWN_STAGGER_S = 10.0    # between robots (each runs the Clearpath generators)
NAV2_DELAY_S = 40.0       # all robots spawned and their controllers active


def include(name, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(INCLUDE, name)),
        launch_arguments=arguments.items())


def launch_setup(context, *args, **kwargs):
    team = robots()
    selection = ','.join(r['namespace'] for r in team)
    yolo = LaunchConfiguration('yolo').perform(context) == 'true'
    return [
        include('sim.launch.py', world=WORLD,
                headless=LaunchConfiguration('headless').perform(context)),
        TimerAction(period=SPAWN_DELAY_S, actions=[include(
            'robots.launch.py', world=WORLD, robots=selection,
            stagger_s=str(SPAWN_STAGGER_S))]),
        include('mapping.launch.py', world=WORLD, robots=selection),
        TimerAction(period=NAV2_DELAY_S, actions=[
            include('nav2.launch.py', namespace=r['namespace']) for r in team]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SHARE, 'launch', 'perception.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in
                              ('yolo_model', 'yolo_device', 'yolo_target')}.items(),
            condition=IfCondition(LaunchConfiguration('yolo'))),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(SHARE, 'rviz',
                       'exploration_yolo26.rviz' if yolo else 'exploration.rviz')],
            parameters=[{'use_sim_time': True}], output='log',
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        SetEnvironmentVariable('GZ_PARTITION', 'field_exploration'),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='run Gazebo without its GUI'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='start RViz'),
        DeclareLaunchArgument('yolo', default_value='true', choices=['true', 'false'],
                              description='start YOLO26 on the CUDA GPU and show annotated RGB panels'),
        DeclareLaunchArgument('yolo_model', default_value=PathJoinSubstitution([
            FindPackageShare('search_perception'), 'models', 'yolo26n.pt'])),
        DeclareLaunchArgument('yolo_device', default_value='0'),
        DeclareLaunchArgument('yolo_target', default_value='red tennis ball'),
        OpaqueFunction(function=launch_setup),
    ])
