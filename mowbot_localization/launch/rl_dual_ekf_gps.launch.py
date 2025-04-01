from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    rlgps_config_path = PathJoinSubstitution(
        # [FindPackageShare('mowbot_localization'), 'config', 'dual_ekf_gps_params.yaml']
        [FindPackageShare('mowbot_localization'), 'config', 'dual_ekf_gps_params_2.yaml']
    )

    return LaunchDescription([  
                              
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        
        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                rlgps_config_path,
                {"use_sim_time": False},
            ],
            remappings=[
                #input
                ("odom", "mowbot_base/odom"),
                ("imu", "mb_imu_heading/data"),
                #output
                ("odometry/filtered", "odometry/local")
            ]
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                rlgps_config_path,
                {"use_sim_time": False},
            ],
            remappings=[  
                #input
                ("odom", "mowbot_base/odom"),
                ("imu", "mb_imu_heading/data"),
                ("odometry/gps", "odometry/gpsr"),
                #output
                ("odometry/filtered", "odometry/global")   
            ]
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                rlgps_config_path,
                {"use_sim_time": False},
            ],
            remappings=[
                #input
                ("odometry/filtered", "odometry/global"),
                ("gps/fix", "/ublox_gpsr_node/fix"),
                ("imu", "mb_imu_heading/data"),
                #output
                ("odometry/gps", "odometry/gpsr" ),
                ("gps/filtered", "gpsr/filtered"), 
    
            ]
        ),
    ])