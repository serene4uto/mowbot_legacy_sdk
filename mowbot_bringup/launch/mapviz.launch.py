from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    mapviz_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'gps_display.mvc']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": mapviz_config_path}],
        ),

        # Node(
        #     package="swri_transform_util",
        #     executable="initialize_origin.py",
        #     name="initialize_origin",
        #     parameters=[
        #         {"local_xy_frame": "map"},
        #         # {"local_xy_origin": "auto"},
        #         {"local_xy_origin": "origin"},
        #         {
        #             "local_xy_origins": [
        #                 36.1142823,
        #                 128.4212675,
        #                 0.0,
        #                 0.0,
        #             ]
        #         },
        #     ],
        #     remappings=[('/fix', '/ublox_gpsr_node/fix')],
        # ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "combined_gps/fix"),
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        ),

    ])