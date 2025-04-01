from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    joy_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'joy.yaml']
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='joy',
            executable='joy_node',
            output='screen',
            name='joy_node',
            parameters=[
                joy_config_path
        ]),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                joy_config_path
            ],
            remappings=[
                ('/cmd_vel', '/joy_cmd_vel')
            ]
        ),
        
    ])