from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml





def generate_launch_description():

    nav2_params = PathJoinSubstitution(
        [FindPackageShare('mowbot_navigation'), 'config', 'nav_no_map_params.yaml']
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    rl_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_localization'), 'config', 'ekf_params.yaml']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace"
        ),

        DeclareLaunchArgument(
            "rviz",
            default_value="False",
            description="Whether to launch RViz"
        ),

        DeclareLaunchArgument(
            "tf",
            default_value="True",
            description="Whether to launch TF"
        ),

        DeclareLaunchArgument(
            name='rl',
            default_value='false',
            description='Use robot_localization'
        ),

        DeclareLaunchArgument(
            "wpfl",
            default_value="False",
            description="Whether to launch waypoints follower"
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration("rl")),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                rl_config_path
            ]
        ),

        # nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
                )
            ),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": configured_params,
                "autostart": "True",
            }.items(),
        ),

        # rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_navigation'), 'launch', 'rviz.launch.py']
                )
            ),
            condition=IfCondition(LaunchConfiguration("rviz"))
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(LaunchConfiguration("tf"))
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("wpfl")),
            package='py_mowbot_utils',
            executable='nav_no_map_wp_follower',
            name='nav_no_map_wp_follower',
            output='screen',
        ),

    ])