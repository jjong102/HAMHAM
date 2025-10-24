#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy,
    QoSReliabilityPolicy, QoSDurabilityPolicy
)
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, Bool          # [MOD] Bool 추가
from builtin_interfaces.msg import Duration
import numpy as np
import math


class LidarRoiFilterNode(Node):
    """
    - 전방 '각도 ROI'로 LaserScan 필터링
    - 유클리드 군집화로 장애물 중심 추정
    - 직사각형 ROI(로봇 프레임) 내부만 별도 시각화
    - ROI 내부 '최소 거리' Float32 발행
    - [NEW] ROI 내부 거리가 alert_dist_m 이하면 Bool 경보 발행(/obstacles/roi_alert)
    """

    def __init__(self):
        super().__init__('lidar_roi__node')

        # ===== 각도 ROI 파라미터 =====
        self.front_angle_deg = 60.0
        self.roi_center_deg  = -90.0
        self.normalize_angle = True
        self.enable_filter   = True

        # ===== 직사각형 ROI (로봇 프레임) =====
        self.rect_x_min = 0.0
        self.rect_x_max = 5.0
        self.rect_y_min = -0.5
        self.rect_y_max =  0.5

        # ===== 군집/판정 파라미터 =====
        self.min_cluster_points   = 3
        self.min_cluster_length_m = 0.03
        self.link_base  = 0.08
        self.link_slope = 0.03
        self.max_gap = 1

        # ===== 시각화/알림 =====
        self.marker_lifetime_sec = 0.3
        self.alert_dist_m = 1.5             # [NEW] 경보 임계값(미터)

        # ===== QoS =====
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # ===== Sub/Pub =====
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, self.qos_profile
        )
        self.pub = self.create_publisher(
            LaserScan, '/scan/front_filtered', self.qos_profile
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/obstacles/centers', self.qos_profile
        )
        self.marker_pub_rect = self.create_publisher(
            MarkerArray, '/obstacles/roi_centers', self.qos_profile
        )
        self.min_dist_pub = self.create_publisher(
            Float32, '/obstacles/roi_min_distance', self.qos_profile
        )
        self.alert_pub = self.create_publisher(               # [NEW]
            Bool, '/obstacles/roi_alert', self.qos_profile
        )

        # 내부 계산용
        self.half_front = math.radians(self.front_angle_deg / 2.0)
        self.roi_center = math.radians(self.roi_center_deg)

        self.get_logger().info(
            f"[INIT] AngROI center={self.roi_center_deg}deg, span=±{self.front_angle_deg/2:.1f}deg | "
            f"RectROI x[{self.rect_x_min},{self.rect_x_max}] y[{self.rect_y_min},{self.rect_y_max}] | "
            f"cluster(min_pts={self.min_cluster_points}, min_len={self.min_cluster_length_m}m, "
            f"link={self.link_base}+{self.link_slope}*r, max_gap={self.max_gap}) | "
            f"Lifetime={self.marker_lifetime_sec}s | alert<= {self.alert_dist_m:.2f} m"
        )

    def scan_callback(self, msg: LaserScan):
        if not self.enable_filter:
            self.pub.publish(msg)
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n = len(msg.ranges)
        eps = 0.5 * angle_inc

        # 1) 전방 각도 ROI 필터
        filtered = []
        valid = 0
        for i, r in enumerate(msg.ranges):
            if np.isnan(r) or np.isinf(r) or r < msg.range_min or r > msg.range_max:
                filtered.append(float('inf'))
                continue
            ang = angle_min + i * angle_inc
            if self.normalize_angle:
                ang = (ang + math.pi) % (2 * math.pi) - math.pi
            ang_rel = (ang - self.roi_center + math.pi) % (2 * math.pi) - math.pi
            if -self.half_front - eps <= ang_rel <= self.half_front + eps:
                filtered.append(r); valid += 1
            else:
                filtered.append(float('inf'))

        # 2) 필터 스캔 퍼블리시
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered
        out.intensities = msg.intensities
        self.pub.publish(out)

        # 3) 클러스터링 + 판정
        centers_scan, centers_robot = self._euclidean_cluster_centers(out)

        #   3-1) 전체 장애물 마커
        self._publish_center_markers(out.header.frame_id, centers_scan)

        #   3-2) 직사각형 ROI 내부만 + 거리 텍스트
        rect_centers_scan, rect_dists = self._select_rect_roi(centers_scan, centers_robot)
        self._publish_rect_markers(out.header.frame_id, rect_centers_scan, rect_dists)

        #   3-3) 최소 거리 발행
        min_dist_msg = Float32()
        if len(rect_dists) > 0:
            min_val = float(np.min(rect_dists))
        else:
            min_val = float('nan')
        min_dist_msg.data = min_val
        self.min_dist_pub.publish(min_dist_msg)

        #   3-4) [NEW] 2 m 이하 경보 Bool 발행
        alert_msg = Bool()
        alert_msg.data = any(d <= self.alert_dist_m for d in rect_dists) if len(rect_dists) > 0 else False
        self.alert_pub.publish(alert_msg)

        # self.get_logger().info(
        #     f"[ROI] span=±{self.front_angle_deg/2:.1f}°, valid={valid}/{n} | "
        #     f"obstacles={len(centers_scan)} | in_rect={len(rect_centers_scan)} | "
        #     f"min_dist_in_rect={min_val if not math.isnan(min_val) else 'NaN'} m | "
        #     f"alert({self.alert_dist_m:.2f}m)=> {alert_msg.data}"
        # )

    # --- (이하 군집화/마커 함수 동일, 생략 없이 유지) ---
    def _euclidean_cluster_centers(self, scan: LaserScan):
        ranges = scan.ranges
        ang_min = scan.angle_min
        ang_inc = scan.angle_increment
        n = len(ranges)

        centers_scan = []
        centers_robot = []

        def neighbor_euclid(i: int, j: int) -> float:
            ri, rj = ranges[i], ranges[j]
            if not (np.isfinite(ri) and np.isfinite(rj)):
                return float('inf')
            dtheta = abs(ang_inc) * (j - i)
            return math.sqrt(max(0.0, ri*ri + rj*rj - 2.0*ri*rj*math.cos(dtheta)))

        start = None
        gap = 0
        last_valid = None

        i = 0
        while i < n:
            if np.isfinite(ranges[i]):
                if start is None:
                    start = i
                if last_valid is not None:
                    d = neighbor_euclid(last_valid, i)
                    r_mean = (ranges[last_valid] + ranges[i]) * 0.5
                    link_thresh = self.link_base + self.link_slope * r_mean
                    if d > link_thresh:
                        self._maybe_add_center_euclid(
                            start, last_valid, ranges, ang_min, ang_inc,
                            centers_scan, centers_robot
                        )
                        start = i
                        gap = 0
                last_valid = i
                gap = 0
                i += 1
            else:
                if start is None:
                    i += 1
                    continue
                gap += 1
                if gap > self.max_gap:
                    self._maybe_add_center_euclid(
                        start, last_valid, ranges, ang_min, ang_inc,
                        centers_scan, centers_robot
                    )
                    start = None
                    last_valid = None
                    gap = 0
                    i += 1
                else:
                    j = i + 1
                    while j < n and not np.isfinite(ranges[j]):
                        j += 1
                    if j < n and np.isfinite(ranges[j]) and last_valid is not None:
                        d = neighbor_euclid(last_valid, j)
                        r_mean = (ranges[last_valid] + ranges[j]) * 0.5
                        link_thresh = self.link_base + self.link_slope * r_mean
                        if d <= link_thresh:
                            i = j
                            continue
                    self._maybe_add_center_euclid(
                        start, last_valid, ranges, ang_min, ang_inc,
                        centers_scan, centers_robot
                    )
                    start = None
                    last_valid = None
                    gap = 0
                    i += 1

        if start is not None and last_valid is not None:
            self._maybe_add_center_euclid(
                start, last_valid, ranges, ang_min, ang_inc,
                centers_scan, centers_robot
            )

        return centers_scan, centers_robot

    def _maybe_add_center_euclid(self, s, e, ranges, ang_min, ang_inc,
                                 centers_scan, centers_robot):
        idxs = [i for i in range(s, e + 1) if np.isfinite(ranges[i])]
        if len(idxs) < self.min_cluster_points:
            return

        i0, i1 = idxs[0], idxs[-1]
        r0, r1 = ranges[i0], ranges[i1]
        th0 = ang_min + i0 * ang_inc
        th1 = ang_min + i1 * ang_inc
        if self.normalize_angle:
            th0 = (th0 + math.pi) % (2 * math.pi) - math.pi
            th1 = (th1 + math.pi) % (2 * math.pi) - math.pi

        x0_s, y0_s = r0 * math.cos(th0), r0 * math.sin(th0)
        x1_s, y1_s = r1 * math.cos(th1), r1 * math.sin(th1)
        chord_len = math.hypot(x1_s - x0_s, y1_s - y0_s)
        if chord_len < self.min_cluster_length_m:
            return

        xs_s, ys_s = [], []
        for i in idxs:
            r = ranges[i]
            th = ang_min + i * ang_inc
            if self.normalize_angle:
                th = (th + math.pi) % (2 * math.pi) - math.pi
            xs_s.append(r * math.cos(th))
            ys_s.append(r * math.sin(th))
        cx_s = float(np.mean(xs_s))
        cy_s = float(np.mean(ys_s))
        centers_scan.append((cx_s, cy_s))

        cos_c = math.cos(-self.roi_center)
        sin_c = math.sin(-self.roi_center)
        cx_r = cos_c * cx_s - sin_c * cy_s
        cy_r = sin_c * cx_s + cos_c * cy_s
        centers_robot.append((cx_r, cy_r))

    def _select_rect_roi(self, centers_scan, centers_robot):
        sel_scan = []
        dists = []
        for (cx_s, cy_s), (cx_r, cy_r) in zip(centers_scan, centers_robot):
            if (self.rect_x_min <= cx_r <= self.rect_x_max and
                self.rect_y_min <= cy_r <= self.rect_y_max):
                sel_scan.append((cx_s, cy_s))
                dists.append(math.hypot(cx_r, cy_r))
        return sel_scan, dists

    def _publish_center_markers(self, frame_id: str, centers_scan):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (cx, cy) in enumerate(centers_scan):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = "obstacle_centers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color.r = 1.0
            m.color.g = 0.3
            m.color.b = 0.0
            m.color.a = 0.9
            m.lifetime = Duration(
                sec=int(self.marker_lifetime_sec),
                nanosec=int((self.marker_lifetime_sec % 1) * 1e9)
            )
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def _publish_rect_markers(self, frame_id: str, centers_scan, dists):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, ((cx, cy), dist) in enumerate(zip(centers_scan, dists)):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = "obstacle_roi"
            m.id = 10000 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.28
            m.scale.y = 0.28
            m.scale.z = 0.28
            m.color.r = 0.0
            m.color.g = 0.9
            m.color.b = 0.9
            m.color.a = 0.95
            m.lifetime = Duration(
                sec=int(self.marker_lifetime_sec),
                nanosec=int((self.marker_lifetime_sec % 1) * 1e9)
            )
            ma.markers.append(m)

            t = Marker()
            t.header.frame_id = frame_id
            t.header.stamp = stamp
            t.ns = "obstacle_roi_text"
            t.id = 20000 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = cx
            t.pose.position.y = cy
            t.pose.position.z = 0.25
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 0.95
            t.text = f"{dist:.2f} m"
            t.lifetime = Duration(
                sec=int(self.marker_lifetime_sec),
                nanosec=int((self.marker_lifetime_sec % 1) * 1e9)
            )
            ma.markers.append(t)
        self.marker_pub_rect.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRoiFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
