from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # ---------------------------------------------
    # Node configuration for ublox_gps
    # ---------------------------------------------
    ublox_node = Node(
        package='ublox_gps',                  # ROS 2 package containing the ublox GPS driver
        executable='ublox_gps_node',          # Executable name for the ublox GPS node
        name='ublox_gps_node',                # Name assigned to the node
        output='screen',                      # Output log to screen
        arguments=['--ros-args', '--log-level', 'ublox_gps_node:=debug'],
        parameters=[{                         # Node parameters defined inline
            'debug': 0,
            'device': '/dev/tty_Ardusimple',           # 실제 포트 확인: ls -l /dev/ttyACM*
            'frame_id': 'gps',
            'config_on_startup': True,
            'uart1': {
                'baudrate': 9600              # Baudrate for UART1
            },
            'tmode3': 0,
            'dynamic_model': 'automotive',


            'rate': 5.0,
            'nav_rate': 1,
            

            'nmea': {
                'set': False
            },

            # ★ 여기! 중첩으로 전달
            'publish': {
                'all': False,
                'nav': {
                    'pvt': True,
                    'hpposllh': True,
                    'sat': True        # NAV-SAT 퍼블리셔 생성
                },
                'rxm': {
                    'rtcm': True
                }
            },

            'gnss': {
                'gps': True,
                'glonass': True,      # ← 추가로 켤 별자리
                'galileo': True,
                'beidou': True,
                'qzss': True,
                'sbas': False
            },
            
            'inf': {
                'all': False,
                'warning': True,
                'error': True
            }
        }]
    )

    # ---------------------------------------------
    # Environment variable to control NTRIP client debug
    # ---------------------------------------------
    set_debug_env = SetEnvironmentVariable(
        name='NTRIP_CLIENT_DEBUG',  # Name of the environment variable
        value='false'               # Disable debug output
    )

    # ---------------------------------------------
    # Node configuration for NTRIP client
    # ---------------------------------------------
    ntrip_node = Node(
        package='ntrip_client',              # ROS 2 package containing the NTRIP client
        executable='ntrip_ros.py',           # Python script for the NTRIP client
        name='ntrip_client',                 # Name assigned to the node
        output='screen',                     # Output log to screen
        parameters=[{                        # Parameters required for NTRIP connection
            'host': 'RTS2.ngii.go.kr',  # NTRIP caster hostname
            'port': 2101,                            # NTRIP port (integer)
            'mountpoint': 'VRS-RTCM32',               # Mountpoint on the NTRIP caster
            'ntrip_version': 'None',                 # Optional NTRIP version
            'authenticate': True,                    # Use authentication (username/password)
            'username': 'cotnqls000_250901',  # Auth username
            'password': 'ngii',              # Auth password
            'ssl': False,                            # SSL not used
            'cert': 'None',                          # No client certificate
            'key': 'None',                           # No client key
            'ca_cert': 'None',                       # No custom CA certificate
            'rtcm_frame_id': 'odom',                 # Frame ID for published RTCM messages
            'nmea_max_length': 128,                  # Max NMEA sentence length
            'nmea_min_length': 3,                    # Min NMEA sentence length
            'rtcm_message_package': 'rtcm_msgs',     # Use the rtcm_msgs message format
            'reconnect_attempt_max': 10,             # Max reconnect attempts before giving up
            'reconnect_attempt_wait_seconds': 5,     # Wait time between reconnects
            'rtcm_timeout_seconds': 4                # Max time without RTCM before reconnect
        }],
        remappings=[
            ('/fix', '/ublox_gps_node/fix')  # Remap /fix topic to /ublox_gps_node/fix
        ]
    )
    imu_node = Node(
        package='combined_rtk',      # 네 패키지명
        executable='bno055',         # setup.py console_scripts: 'bno055 = combined_rtk.bno055:main'
        name='bno055',               # node 이름, 이렇게 해도 됨
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'baud': 9600,
            'topic_yaw': 'bno055/yaw_deg',
            'topic_cal': 'bno055/calibration',
            'topic_ready': 'bno055/is_calibrated',
            'topic_debug': 'bno055/serial_debug',
            'to_radians': False,
            'arming_delay_ms': 300
        }]
    )

    # Return the full launch description with all configured actions
    return LaunchDescription([
        set_debug_env,  # Set environment variable for NTRIP debug
        ublox_node,     # Launch ublox GPS node
        ntrip_node,      # Launch NTRIP client node
        imu_node
    ])