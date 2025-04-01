from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    twist_mux_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'twist_mux.yaml']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        DeclareLaunchArgument(
            'use_teleop_joy',
            default_value='true',
            description='Use teleop joy'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_bringup'), 'launch', 'teleop.launch.py']
                )
            ),
            condition=IfCondition(LaunchConfiguration('use_teleop_joy')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace')
            }.items()
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            name='twist_mux',
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={
                ('/cmd_vel_out', '/mowbot_base/cmd_vel_unstamped')
            },
            parameters=[
                twist_mux_config_path
            ]
        ),
    ])