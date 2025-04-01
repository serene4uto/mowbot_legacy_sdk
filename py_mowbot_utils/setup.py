from setuptools import find_packages, setup

package_name = 'py_mowbot_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_gps_compass = py_mowbot_utils.dual_gps_compass:main',
            'gps_waypoints_follower = py_mowbot_utils.gps_waypoints_follower:main',
            'gps_waypoints_planner = py_mowbot_utils.gps_waypoints_planner:main',
            'gps_waypoints_logger = py_mowbot_utils.gps_waypoints_logger:main',

            '2d_slam_map_saver = py_mowbot_utils.2d_slam_map_saver:main',
            'nav_no_map_wp_follower = py_mowbot_utils.nav_no_map_wp_follower:main',
            'system_monitor = py_mowbot_utils.system_monitor:main',
            'sensor_monitor = py_mowbot_utils.sensor_monitor:main',
            
            'ktserver_client = ktserver_demo.ktserver_client:main',
            
        ],
    },
)
