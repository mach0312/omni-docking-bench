"""Kinematic (convergence) controller experiment launch.

CLI overrides:
  heading_offsets_deg:=[0.0,-3.0,3.0]
  repeats:=3
  results_dir:=/path/to/out
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_float_list(s: str):
    s = s.strip().lstrip('[').rstrip(']')
    if not s:
        return [0.0]
    return [float(x.strip()) for x in s.split(',') if x.strip()]


def launch_setup(context, *args, **kwargs):
    default_results = os.path.expanduser(
        '~/omni_docking_bench_ws/src/icros_test_setup/results'
    )

    heading_offsets_str = LaunchConfiguration('heading_offsets_deg').perform(context)
    repeats = int(LaunchConfiguration('repeats').perform(context))
    results_dir = LaunchConfiguration('results_dir').perform(context) or default_results
    heading_offsets = _parse_float_list(heading_offsets_str)

    pgv_controller_params = PathJoinSubstitution([
        FindPackageShare('pgv_guided_controller'),
        'config',
        'pgv_controller.params.yaml',
    ])

    sensor_offset_x = 0.00
    sensor_offset_y = 0.51

    experiment_runner = Node(
        package='icros_test_setup',
        executable='experiment_runner_node',
        name='experiment_runner',
        parameters=[{
            'controller_name': 'kinematic',
            'pose_topic': '/pgv/raw_pose',
            'repeats': repeats,
            'heading_offsets_deg': heading_offsets,
            'results_dir': results_dir,
            'sensor_offset_x': sensor_offset_x,
            'sensor_offset_y': sensor_offset_y,
        }],
        output='screen',
    )

    return [
        Node(
            package='icros_test_setup',
            executable='dummy_pgv_node',
            name='dummy_pgv_node',
            parameters=[{
                'odom_topic': '/dsdbot_base_controller/odom',
                'pose_topic': '/pgv/raw_pose',
                'sim_sensor_offset_x': 0.0,
                'sim_sensor_offset_y': sensor_offset_y,
                'init_x': 0.0,
                'init_y': -sensor_offset_y,
                'publish_tf': True,
            }],
            output='screen',
        ),
        Node(
            package='pgv_guided_controller',
            executable='pgv_controller',
            name='pgv_controller',
            parameters=[pgv_controller_params],
            output='screen',
        ),
        experiment_runner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=experiment_runner,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'heading_offsets_deg',
            default_value='[0.0,-2.0,2.0,-3.0,3.0,-4.0,4.0,-5.0,5.0]',
            description='Heading offset list (deg). Example: "[0.0,-3.0,3.0]"',
        ),
        DeclareLaunchArgument(
            'repeats',
            default_value='2',
            description='Repeats per (direction, heading_offset)',
        ),
        DeclareLaunchArgument(
            'results_dir',
            default_value='',
            description='Output CSV directory. Empty = default.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
