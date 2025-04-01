from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    nav2_params = PathJoinSubstitution(
        [FindPackageShare('mowbot_navigation'), 'config', 'nav_2d_map_params.yaml']
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    mapper_params = PathJoinSubstitution(
        [FindPackageShare('mowbot_navigation'), 'config', 'mapper_params_online_async.yaml']
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
            name='rl',
            default_value='false',
            description='Use robot_localization'
        ),

        # Robot Localization (RL)
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

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
                )
            ),
            launch_arguments={
                "use_sim_time": "false",
                "slam_params_file": mapper_params,
            }.items(),
        ),

        # Nav2 (delayed using TimerAction)
        TimerAction(
            period=1.0,  
            actions=[
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

                # RViz
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [FindPackageShare('mowbot_navigation'), 'launch', 'rviz.launch.py']
                        )
                    ),
                    condition=IfCondition(LaunchConfiguration("rviz"))
                ),

                # Node(
                #     namespace=LaunchConfiguration('namespace'),
                #     package='py_mowbot_utils',
                #     executable='2d_slam_map_saver',
                #     output='screen'
                # ),
            ]
        ),



    ])
