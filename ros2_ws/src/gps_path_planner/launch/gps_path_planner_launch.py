from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Global Path Publisher
        Node(
            package='gps_path_planner',
            executable='gps_publish_global_path_node',
            name='gps_publish_global_path_node',
            output='screen'
        ),

        # Pure Pursuit (조향 계산)
        Node(
            package='gps_path_planner',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen'
        ),

        # Pose Publisher (/current_xy 등 위치 퍼블리시)
        Node(
            package='gps_path_planner',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen'
        ),

        # Twist Serial Bridge (조향은 /cmd_vel.angular.z, 속도는 /drive_throttle)
        Node(
            package='gps_path_planner',
            executable='twist_serial_bridge',
            name='twist_serial_bridge',
            output='screen',
            parameters=[{
                'twist_topic': '/cmd_vel',
                'max_speed_mps': 2.0,
                'max_steer_rad': 0.35,
                'wheelbase_m': 0.7,
                'cmd_is_steer_angle': True,
                'invert_steer': False,
                'deadman_timeout_s': 0.5,
                'throttle_topic': '/drive_throttle',
                'throttle_default_pct': 0
            }]
        ),
        # ✅ 위·경도 트리거 → /drive_throttle 퍼센트 발행 (조향은 건드리지 않음)
        Node(
            package='gps_path_planner',
            executable='mission_manager',   # ← setup.py 콘솔 스크립트 이름
            name='geo_drive_throttle',
            output='screen'
        ),

        # ───────── Plan C: 카메라를 퍼블리시하고, 추론 노드는 이미지 구독 ─────────
        # 왼쪽 카메라 퍼블리셔 (/left_cam/image_raw)
        Node(
            package='gps_path_planner',
            executable='camera_publisher',
            name='left_camera',
            namespace='left_cam',
            output='screen',
            parameters=[{
                'device_index': 6,
                'image_topic': 'image_raw',
                'show_window': False
            }]
        ),
        # 중앙 카메라 퍼블리셔 (/center_cam/image_raw)
        Node(
            package='gps_path_planner',
            executable='camera_publisher',
            name='center_camera',
            namespace='center_cam',
            output='screen',
            parameters=[{
                'device_index': 4,
                'image_topic': 'image_raw',
                'show_window': False
            }]
        ),

        # 왼쪽 cone detector (입력: /left_cam/image_raw)
        Node(
            package='gps_path_planner',
            executable='cone_detector',
            name='cone_left',
            namespace='left_cam',
            output='screen',
            parameters=[{
                'image_in': 'image_raw',
                'show_window': False
            }],
            remappings=[
                ('/cone/state', '/cones/left/state'),
                ('/lane/outline', '/cones/left/image'),
            ]
        ),

        # 중앙 cone detector (입력: /center_cam/image_raw)
        Node(
            package='gps_path_planner',
            executable='cone_detector',
            name='cone_center',
            namespace='center_cam',
            output='screen',
            parameters=[{
                'image_in': 'image_raw',
                'show_window': False
            }],
            remappings=[
                ('/cone/state', '/cones/center/state'),
                ('/lane/outline', '/cones/center/image'),
            ]
        ),

        # 중앙 traffic light detector (입력: /center_cam/image_raw)
        Node(
            package='gps_path_planner',
            executable='traffic_light_node',
            name='traffic_light_center',
            namespace='center_cam',
            output='screen',
            parameters=[{
                'image_in': 'image_raw',
                'show_window': False
            }],
            remappings=[
                ('/lane/outline', '/traffic_light/image'),
            ]
        ),

        # ✅ lidar_roi_node 실행
        Node(
            package='gps_path_planner',
            executable='lidar_roi_node',   # setup.py 의 console_scripts 이름과 동일
            name='lidar_roi_node',         # 런치에서 표시될 ROS 노드명
            output='screen',
            # parameters=[{'param_name': 'value'}]  # 필요 시 파라미터 지정 가능
        ),
    ])
