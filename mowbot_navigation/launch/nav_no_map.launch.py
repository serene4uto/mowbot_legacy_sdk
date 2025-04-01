from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    nav_dir = get_package_share_directory('mowbot_navigation')
    nav_param_file = os.path.join(nav_dir, 'param', 'nav2_no_map_params.yaml')
    rviz_config_file = os.path.join(nav_dir, 'rviz', 'nav2_nav_no_map_view.rviz')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    declare_use_rviz_argument = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_param_file
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map'],
        parameters=[{'use_sim_time': use_sim_time}],
        # remappings=[('tf', 'tf')],
    )
    

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_use_rviz_argument,
        nav2_bringup_launch,
        rviz_node,
        tf2_node
    ])