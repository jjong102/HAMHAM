#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import pyproj

# 한국 서쪽 지역 예시: 자기편각 약 -8도
MAG_DECLINATION_DEG = -8.0
# UTM 좌표 변환기 (한국은 보통 zone 52 사용)
proj_utm = pyproj.Proj(proj='utm', zone=52, ellps='WGS84')


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Subscriber
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)
        self.create_subscription(Float32, 'bno055/yaw_deg', self.yaw_callback, 10)

        # Publisher
        self.pub_xy = self.create_publisher(Pose2D, '/current_xy', 10)      # UTM 좌표
        self.pub_yaw = self.create_publisher(Float32, '/current_yaw', 10)   # yaw(deg)

        # 상태 저장
        self.last_lat = None
        self.last_lon = None
        self.last_yaw_deg = None  # deg

    def gps_callback(self, msg: NavSatFix):
        self.last_lat = msg.latitude
        self.last_lon = msg.longitude
        self.publish_xy()

    def yaw_callback(self, msg: Float32):
        self.last_yaw_deg = msg.data
        self.publish_yaw()

    def publish_xy(self):
        if self.last_lat is None or self.last_lon is None:
            return

        # 위경도 → UTM 변환
        x, y = proj_utm(self.last_lon, self.last_lat)

        xy_msg = Pose2D()
        xy_msg.x = float(x)
        xy_msg.y = float(y)
        xy_msg.theta = 0.0  # Pure Pursuit에서 필요 없으므로 0
        self.pub_xy.publish(xy_msg)

    def publish_yaw(self):
        if self.last_yaw_deg is None:
            return

        # 센서 방향 반전 + 자기편각 보정
        yaw_corrected_deg = -self.last_yaw_deg + MAG_DECLINATION_DEG

        # 0~360 범위로 wrap
        yaw_corrected_deg = yaw_corrected_deg % 360

        yaw_msg = Float32()
        yaw_msg.data = float(yaw_corrected_deg)
        self.pub_yaw.publish(yaw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
