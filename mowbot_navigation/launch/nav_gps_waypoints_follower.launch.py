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
        [FindPackageShare('mowbot_navigation'), 'config', 'gps_nav_no_maps_params.yaml']
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    mapviz_config = PathJoinSubstitution(
        [FindPackageShare('mowbot_navigation'), 'config', 'gps_wpf.mvc']
    )

    
    return LaunchDescription([

        DeclareLaunchArgument(
            "rviz",
            default_value="False",
            description="Whether to launch RViz"
        ),

        DeclareLaunchArgument(
            "mapviz",
            default_value="False",
            description="Whether to launch MapViz"
        ),

        DeclareLaunchArgument(
            "rl",
            default_value="False",
            description="Whether to launch Robot Localization"
        ),
        
        # robot localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_localization'), 'launch', 'rl_dual_ekf_navsat.launch.py']
                )
            ),
            condition=IfCondition(LaunchConfiguration("rl"))
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

        # mapviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_navigation'), 'launch', 'mapviz.launch.py']
                )
            ),
            launch_arguments={
                "fix_topic": "combined_gps/fix",
                "mvc_config": mapviz_config,
            }.items(),
            condition=IfCondition(LaunchConfiguration("mapviz"))
        ),

        Node(
            package="py_mowbot_utils",
            executable="gps_waypoints_follower",
            name="gps_waypoints_follower",
            output="screen",
        ),

    ])





    