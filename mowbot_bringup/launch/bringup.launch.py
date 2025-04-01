from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    extra_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'extra.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'rviz', 'mowbot.rviz']
    )

    rl_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_localization'), 'config', 'ekf_params.yaml']
    )

    rlgps_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_localization'), 'config', 'dual_ekf_gps_params.yaml']
    )

    return LaunchDescription([  

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Whether to start rviz'
        ),

        DeclareLaunchArgument(
            'imu',
            default_value='false',
            description='Whether to start the imu'
        ),

        DeclareLaunchArgument(
            'laser',
            default_value='false',
            description='Whether to start the laser'
        ),

        DeclareLaunchArgument(
            'dcam',
            default_value='false',
            description='Whether to start the depth camera'
        ),

        DeclareLaunchArgument(
            name='madgwick',
            default_value='false',
            description='Use madgwick to fuse imu and magnetometer'
        ),

        DeclareLaunchArgument(
            name='rl',
            default_value='false',
            description='Use robot_localization'
        ),

        DeclareLaunchArgument(
            name='rlgps',
            default_value='false',
            description='Use robot_localization with gps'
        ),

        DeclareLaunchArgument(
            name='uros',
            default_value='false',
            description='Use micro-ros'
        ),
        
        DeclareLaunchArgument(
            name='uros_serial_port',
            default_value='/dev/MBB-UROS',
            description='Serial port for uros communication'
        ),

        DeclareLaunchArgument(
            name='uros_baudrate',
            default_value='115200',
            description='Baudrate for uros serial communication'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("uros_serial_port"), '--baudrate', LaunchConfiguration("uros_baudrate")],
            condition=IfCondition(LaunchConfiguration("uros"))
        ),

        DeclareLaunchArgument(
            'ntrip',
            default_value='false',
            description='Whether to start the ntrip client'
        ),

        DeclareLaunchArgument(
            'gpsl',
            default_value='false',
            description='Whether to start the left gps'
        ),

        DeclareLaunchArgument(
            'gpsr',
            default_value='false',
            description='Whether to start the right gps'
        ),

        DeclareLaunchArgument(
            'dgps_compass',
            default_value='false',
            description='Whether to start the dual gps compass'
        ),

        DeclareLaunchArgument(
            'foxglove',
            default_value='false',
            description='Whether to start the foxglove'
        ),

        DeclareLaunchArgument(
            'sensormon',
            default_value='false',
            description='Whether to start the sensor monitor'
        ),


        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_description'), 'launch', 'description.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': 'false'
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_bringup'), 'launch', 'twist_control.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace')
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_bringup'), 'launch', 'sensors.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'imu': LaunchConfiguration('imu'),
                'laser': LaunchConfiguration('laser'),
                'dcam': LaunchConfiguration('dcam'),
                'ntrip': LaunchConfiguration('ntrip'),
                'gpsl': LaunchConfiguration('gpsl'),
                'gpsr': LaunchConfiguration('gpsr'),
            }.items()
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration("madgwick")),
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_node',
            output='screen',
            parameters=[extra_config_path],
            remappings=[
                ('imu/data_raw', 'mb_imu/data'),
                ('imu/mag', '/mb_imu/mag')
            ]
        ),

        #Perception
        GroupAction(
            actions=[
                Node(
                    namespace=LaunchConfiguration('namespace'),
                    package='py_mowbot_utils',
                    executable='dual_gps_compass',
                    name='dual_gps_compass',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('dgps_compass')),
                )
            ]
        ),

        # robot_localization
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

        GroupAction(
            actions = [
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
                        ("odom", "/mowbot_base/odom"),
                        ("imu", "imu/data"),
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
                        ("odom", "/mowbot_base/odom"),
                        ("imu", "imu/data"),
                        ("odometry/gps", "odometry/gpsr"),
                        #output
                        ("odometry/filtered", "odometry/global")
                    ]
                ),

                Node(
                    namespace=LaunchConfiguration('namespace'),
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform_node',
                    output='screen',
                    parameters=[
                        rlgps_config_path,
                        {"use_sim_time": False},
                    ],
                    remappings=[
                        #input
                        ("odometry/filtered", "odometry/global"),
                        ("gps/fix", "/ublox_gpsr_node/fix"),
                        ("imu", "imu/data"),
                        #output
                        ("odometry/gps", "odometry/gpsr" ),
                        ("gps/filtered", "gpsr/filtered"),
                    ]
                ),

            ],
            condition=IfCondition(LaunchConfiguration("rlgps"))
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='py_mowbot_utils',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('sensormon')),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml']
            ),
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),

        Node(
            namespace = LaunchConfiguration('namespace'),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config_path]
        ),
    ])