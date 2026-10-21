"""YOLO26 over the team's RGB cameras; can also attach to an existing simulation."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'include'))
from search_common import SHARE, conda_python, robots  # noqa: E402


def launch_setup(context, *args, **kwargs):
    return [Node(
        package='search_perception', executable='yolo26_detector', name='yolo26_detector',
        prefix=[conda_python()],
        parameters=[os.path.join(SHARE, 'config', 'yolo26.yaml'), {
            'robot_namespaces': [r['namespace'] for r in robots()],
            'model_path': ParameterValue(LaunchConfiguration('yolo_model'), value_type=str),
            'device': ParameterValue(LaunchConfiguration('yolo_device'), value_type=int),
            'target_class': ParameterValue(LaunchConfiguration('yolo_target'), value_type=str),
        }], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        DeclareLaunchArgument('yolo_model', default_value=PathJoinSubstitution([
            FindPackageShare('search_perception'), 'models', 'yolo26n.pt'])),
        DeclareLaunchArgument('yolo_device', default_value='0',
                              description='CUDA GPU index; CPU inference is not enabled'),
        DeclareLaunchArgument('yolo_target', default_value='red tennis ball',
                              description='Red tennis ball (sports ball + red filter), or exact YOLO class; empty disables stopping'),
        OpaqueFunction(function=launch_setup),
    ])
