from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('pgv_guided_controller'),
        'config',
        'pgv_controller.params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML file with ros__parameters'),

        Node(
            package='pgv_guided_controller',
            executable='pgv_controller',
            namespace=LaunchConfiguration('namespace'),
            name='pgv_controller',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        )
    ])
