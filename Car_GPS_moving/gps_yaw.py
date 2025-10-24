#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, time, collections
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64

EARTH_R = 6378137.0  # WGS84

def wrap_pi(a: float) -> float:
    while a > math.pi:  a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

def latlon_to_xy_m(lat, lon, lat0, lon0):
    latr = math.radians(lat); lonr = math.radians(lon)
    lat0r = math.radians(lat0); lon0r = math.radians(lon0)
    x = (lonr - lon0r) * math.cos(lat0r) * EARTH_R  # East
    y = (latr - lat0r) * EARTH_R                    # North
    return x, y

def yaw_from_quat(qx, qy, qz, qw):
    # ZYX (yaw-pitch-roll), yaw about +Z
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

class YawInitFromGPS(Node):
    """
    1) 시작 후 직진(누적 1m) 구간에서 GPS로 진행벡터(헤딩) 추정
    2) IMU의 현재 yaw와의 차이를 오프셋으로 계산해 /imu_yaw_offset_rad 발행
    - 좋은 GPS(공분산) & 직진(저 yaw-rate, 충분한 속도)에서만 샘플 사용
    """
    def __init__(self):
        super().__init__("yaw_init_from_gps")

        # ---- Params ----
        self.declare_parameter("fix_topic", "/ublox_gps_node/fix")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("offset_topic", "/imu_yaw_offset_rad")

        self.declare_parameter("min_travel_m", 1.0)          # 누적 이동 거리 기준
        self.declare_parameter("ema_alpha", 0.3)              # GPS 좌표 EMA 평활
        self.declare_parameter("min_speed_mps", 0.5)          # 속도 하한
        self.declare_parameter("max_yawrate_rad_s", 0.3)      # 회전 속도 상한(직진성)
        self.declare_parameter("max_pos_std_m", 5.0)          # GPS 표준편차 상한(공분산 기반)
        self.declare_parameter("publish_rate_hz", 2.0)        # 보정 완료 후 오프셋 재발행 주기
        self.declare_parameter("require_fix", True)           # NavSatStatus 유효 고정 요구

        self.fix_topic    = self.get_parameter("fix_topic").get_parameter_value().string_value
        self.imu_topic    = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.offset_topic = self.get_parameter("offset_topic").get_parameter_value().string_value

        self.min_travel_m = float(self.get_parameter("min_travel_m").value)
        self.ema_alpha    = float(self.get_parameter("ema_alpha").value)
        self.min_speed    = float(self.get_parameter("min_speed_mps").value)
        self.max_yawrate  = float(self.get_parameter("max_yawrate_rad_s").value)
        self.max_pos_std  = float(self.get_parameter("max_pos_std_m").value)
        self.pub_rate_hz  = float(self.get_parameter("publish_rate_hz").value)
        self.require_fix  = bool(self.get_parameter("require_fix").value)

        # ---- State ----
        self.origin_set = False
        self.lat0 = None; self.lon0 = None

        self.have_imu = False
        self.imu_yaw = 0.0
        self.imu_yawrate = 0.0

        self.x_ema = None; self.y_ema = None
        self.last_t = None
        self.total_dist = 0.0
        self.start_x = None; self.start_y = None
        self.samples = collections.deque(maxlen=200)

        self.calibrated = False
        self.yaw_offset = 0.0
        self._last_pub = 0.0

        # ---- ROS I/O ----
        self.sub_fix = self.create_subscription(NavSatFix, self.fix_topic, self.cb_fix, 10)
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 50)
        self.pub_off = self.create_publisher(Float64, self.offset_topic, 1)

        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz 내부 루프

        self.get_logger().info(
            f"YawInitFromGPS: fix={self.fix_topic} imu={self.imu_topic} → offset={self.offset_topic} | "
            f"min_travel={self.min_travel_m}m, min_speed={self.min_speed}m/s, max_yawrate={self.max_yawrate}rad/s"
        )

    # ---- Callbacks ----
    def cb_imu(self, msg: Imu):
        # orientation → yaw
        q = msg.orientation
        self.imu_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        # yaw-rate
        self.imu_yawrate = float(msg.angular_velocity.z)
        self.have_imu = True

    def _gps_quality_ok(self, msg: NavSatFix) -> bool:
        if self.require_fix and msg.status.status < 0:  # STATUS_NO_FIX=-1
            return False
        cov = list(msg.position_covariance)
        # 대각 성분(variance) → 표준편차(m). 0이면 정보 없음 → 패스(허용)
        try:
            sx = math.sqrt(cov[0]) if cov[0] > 0 else 0.0
            sy = math.sqrt(cov[4]) if cov[4] > 0 else 0.0
            if max(sx, sy) > self.max_pos_std:
                return False
        except Exception:
            pass
        return True

    def cb_fix(self, msg: NavSatFix):
        if not self._gps_quality_ok(msg):
            return

        lat = float(msg.latitude); lon = float(msg.longitude)
        tnow = self.get_clock().now().nanoseconds * 1e-9

        if not self.origin_set:
            self.lat0 = lat; self.lon0 = lon
            self.origin_set = True
            x, y = 0.0, 0.0
            self.x_ema, self.y_ema = x, y
            self.start_x, self.start_y = x, y
            self.last_t = tnow
            self.samples.clear()
            self.total_dist = 0.0
            return

        x, y = latlon_to_xy_m(lat, lon, self.lat0, self.lon0)

        # EMA smoothing
        a = max(0.0, min(1.0, self.ema_alpha))
        self.x_ema = a * x + (1.0 - a) * self.x_ema
        self.y_ema = a * y + (1.0 - a) * self.y_ema

        # 누적 거리 & 속도
        if self.samples:
            px, py, pt = self.samples[-1]
            dd = math.hypot(self.x_ema - px, self.y_ema - py)
            dt = max(1e-3, tnow - pt)
            spd = dd / dt
        else:
            dd, spd = 0.0, 0.0

        self.samples.append((self.x_ema, self.y_ema, tnow))
        self.total_dist = math.hypot(self.x_ema - self.start_x, self.y_ema - self.start_y)

        # 조건: 충분한 속도 & 낮은 yaw-rate (직진)에서만 accumulate 유지
        if (spd < self.min_speed) or (self.have_imu and abs(self.imu_yawrate) > self.max_yawrate):
            # 조건 불량이면 초기화하여 직진 구간만 사용
            self.start_x, self.start_y = self.x_ema, self.y_ema
            self.total_dist = 0.0
            # 시작점만 남기고 샘플 슬림화
            self.samples.clear()
            self.samples.append((self.x_ema, self.y_ema, tnow))
            return

        # 보정 완료?
        if not self.calibrated and self.total_dist >= self.min_travel_m:
            # GPS 헤딩: 시작점 → 현재 EMA
            dx = self.x_ema - self.start_x
            dy = self.y_ema - self.start_y
            gps_heading = math.atan2(dy, dx)

            # IMU yaw 스냅샷
            imu_yaw_now = self.imu_yaw if self.have_imu else 0.0

            # 오프셋 = GPS헤딩 - IMU yaw (IMU + offset = GPS 정렬)
            self.yaw_offset = wrap_pi(gps_heading - imu_yaw_now)
            self.calibrated = True
            self._last_pub = 0.0  # 즉시 1회 발행되도록

            self.get_logger().info(
                f"[CALIB] success: gps_heading={gps_heading:.3f} rad, imu_yaw={imu_yaw_now:.3f} rad → "
                f"yaw_offset={self.yaw_offset:.3f} rad ({math.degrees(self.yaw_offset):.1f}°)"
            )

    # ---- Timer ----
    def on_timer(self):
        if not self.calibrated:
            return
        now = time.time()
        period = 1.0 / max(1e-3, self.pub_rate_hz)
        if (now - self._last_pub) >= period:
            self._last_pub = now
            self.pub_off.publish(Float64(data=float(self.yaw_offset)))

def main():
    rclpy.init()
    node = None
    try:
        node = YawInitFromGPS()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
