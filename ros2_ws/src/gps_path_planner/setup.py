from setuptools import find_packages, setup

package_name = 'gps_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/gps_path_planner_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leeseojin',
    maintainer_email='seojin1106@inu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_latlon_logger = gps_path_planner.gps_latlon_logger:main',
            'gps_publish_global_path_node = gps_path_planner.gps_publish_global_path_node:main',
            'pure_pursuit_node = gps_path_planner.pure_pursuit_node:main',
            'pose_publisher = gps_path_planner.pose_publisher:main',
            'twist_serial_bridge = gps_path_planner.twist_serial_bridge:main',
            'mission_manager = gps_path_planner.mission_manager:main',
            'cone_detector = gps_path_planner.cone_detector:main',
            'traffic_light_node = gps_path_planner.traffic_light_node:main',
            'camera_publisher = gps_path_planner.camera_publisher:main',
            'lidar_roi_node = gps_path_planner.lidar_roi_node:main',
            # 'realtime_phone = gps_path_planner.realtime_phone:main',
        ],
    },
)
