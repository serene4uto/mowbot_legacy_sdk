# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():

#     bringup_dir = get_package_share_directory('nav2_bringup')
#     launch_dir = os.path.join(bringup_dir, 'launch')


#     bringup_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py'))
#         launch_arguments={
#             'namespace': '',
#             'use_namespace': 'False',
#             'slam': 'False',
#             'map_yaml_file': '',
#             'use_sim_time': 'False',
#             'params_file': '',
#             'autostart': 'True',
#             'use_composition': 'True',
#             'use_respawn': 'False',
#             'log'
#         }.items()
#     )

#     return LaunchDescription([

#     ])