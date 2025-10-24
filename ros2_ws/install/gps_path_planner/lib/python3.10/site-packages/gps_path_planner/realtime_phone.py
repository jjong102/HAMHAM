#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64, Float32

EARTH_R = 6378137.0  # WGS84

def latlon_to_xy_m(lat, lon, lat0, lon0):
    latr = math.radians(lat); lonr = math.radians(lon)
    lat0r = math.radians(lat0); lon0r = math.radians(lon0)
    x = (lonr - lon0r) * math.cos(lat0r) * EARTH_R  # East
    y = (latr - lat0r) * EARTH_R                    # North
    return x, y

def yaw_from_quat(qx, qy, qz, qw):
    # ZYX yaw(+Z)
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_deg180(d: float) -> float:
    while d >= 180.0: d -= 360.0
    while d <  -180.0: d += 360.0
    return d

class RouteBgAndPoseViz(Node):
    """
    CSV 경로(위경도) → ENU(LineStrip) + 현재 GPS 점(SPHERE)
    + (옵션) /current_pose 화살표 + yaw_deg(절대각) 화살표
    - 퍼블리시: /viz/route_markers (MarkerArray)
    """
    def __init__(self):
        super().__init__('route_bg_and_pose_viz')

        # ------------ 파라미터 ------------
        self.declare_parameter('route_csv', '')                 # 한 줄: "lat lon" (공백/쉼표 허용, # 주석)
        self.declare_parameter('frame_id', 'map')               # RViz Fixed Frame과 동일
        self.declare_parameter('fix_topic', '/ublox_gps_node/fix')
        self.declare_parameter('pose_topic', '/current_pose')

        # 원점 선택: fix | csv_first | manual
        self.declare_parameter('origin_mode', 'fix')
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)

        # 경로 라인/마커 스타일
        self.declare_parameter('route_width', 0.12)
        self.declare_parameter('route_color_r', 0.8)
        self.declare_parameter('route_color_g', 0.8)
        self.declare_parameter('route_color_b', 0.8)
        self.declare_parameter('route_color_a', 1.0)

        # 현재 pose 화살표 (옵션)
        self.declare_parameter('show_pose_marker', False)
        self.declare_parameter('arrow_len', 1.2)
        self.declare_parameter('arrow_thickness', 0.12)
        self.declare_parameter('text_height', 0.5)

        # GPS 현재 위치 점
        self.declare_parameter('show_gps_marker', True)
        self.declare_parameter('gps_diameter', 0.5)
        self.declare_parameter('gps_color_r', 1.0)
        self.declare_parameter('gps_color_g', 0.3)
        self.declare_parameter('gps_color_b', 0.3)
        self.declare_parameter('gps_color_a', 1.0)

        # yaw(deg) 구독/표시 설정
        self.declare_parameter('yaw_topic', '/current_yaw')         # std_msgs Float32 또는 Float64
        self.declare_parameter('yaw_zero_is_east', True)        # True: 동=0°, False: 북=0°
        self.declare_parameter('yaw_positive_ccw', True)        # True: 반시계+, False: 시계+
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('yaw_arrow_len', 1.5)
        self.declare_parameter('yaw_color_r', 1.0)
        self.declare_parameter('yaw_color_g', 1.0)
        self.declare_parameter('yaw_color_b', 0.0)
        self.declare_parameter('yaw_color_a', 1.0)

        # (선택) 강제 타입 지정: "float64" / "float32" / ""(auto)
        self.declare_parameter('yaw_force_type', '')

        # ------------ 파라미터 로드 ------------
        self.route_csv   = self.get_parameter('route_csv').get_parameter_value().string_value
        self.frame_id    = self.get_parameter('frame_id').get_parameter_value().string_value
        self.fix_topic   = self.get_parameter('fix_topic').get_parameter_value().string_value
        self.pose_topic  = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.origin_mode = self.get_parameter('origin_mode').get_parameter_value().string_value.lower()
        self.origin_lat  = float(self.get_parameter('origin_lat').value)
        self.origin_lon  = float(self.get_parameter('origin_lon').value)

        self.route_width = float(self.get_parameter('route_width').value)
        self.route_color = (
            float(self.get_parameter('route_color_r').value),
            float(self.get_parameter('route_color_g').value),
            float(self.get_parameter('route_color_b').value),
            float(self.get_parameter('route_color_a').value),
        )

        self.show_pose_marker = bool(self.get_parameter('show_pose_marker').value)
        self.arrow_len   = float(self.get_parameter('arrow_len').value)
        self.arrow_thick = float(self.get_parameter('arrow_thickness').value)
        self.text_h      = float(self.get_parameter('text_height').value)

        self.show_gps_marker = bool(self.get_parameter('show_gps_marker').value)
        self.gps_diam = float(self.get_parameter('gps_diameter').value)
        self.gps_color = (
            float(self.get_parameter('gps_color_r').value),
            float(self.get_parameter('gps_color_g').value),
            float(self.get_parameter('gps_color_b').value),
            float(self.get_parameter('gps_color_a').value),
        )

        self.yaw_topic = self.get_parameter('yaw_topic').get_parameter_value().string_value
        self.yaw_zero_is_east = bool(self.get_parameter('yaw_zero_is_east').value)
        self.yaw_positive_ccw = bool(self.get_parameter('yaw_positive_ccw').value)
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)
        self.yaw_arrow_len = float(self.get_parameter('yaw_arrow_len').value)
        self.yaw_color = (
            float(self.get_parameter('yaw_color_r').value),
            float(self.get_parameter('yaw_color_g').value),
            float(self.get_parameter('yaw_color_b').value),
            float(self.get_parameter('yaw_color_a').value),
        )
        self.yaw_force_type = self.get_parameter('yaw_force_type').get_parameter_value().string_value.lower().strip()

        # ------------ 상태 ------------
        self.route_ll = self._load_route_latlon(self.route_csv) if self.route_csv else []
        if not self.route_ll:
            self.get_logger().warn("route_csv 비었거나 읽기 실패 → 경로 라인 없음")
        self.origin_set = False
        self.lat0 = None; self.lon0 = None

        self.route_xy_ready = False
        self.route_xy = []  # [(x,y), ...]

        self.cur_pose = None
        self.pose_frame_id = None
        self.pose_frame_warned = False

        self.last_fix_ll = None
        self.last_fix_xy = None

        self.last_yaw_deg = None     # 동=0°, CCW(+), [-180,180)
        self._yaw_logged_once = False
        self._yaw_sub = None
        self._yaw_type = None         # "float64" or "float32"
        self._last_yaw_stamp = 0.0

        # ------------ ROS I/O ------------
        if self.origin_mode in ('fix', 'csv_first'):
            self.sub_fix = self.create_subscription(NavSatFix, self.fix_topic, self.cb_fix, qos_profile_sensor_data)
        self.sub_pose = self.create_subscription(PoseStamped, self.pose_topic, self.cb_pose, qos_profile_sensor_data)

        # yaw 구독은 런타임 탐지로 설정
        self._setup_yaw_subscription()

        self.pub_markers = self.create_publisher(MarkerArray, '/viz/route_markers', 2)
        self.timer = self.create_timer(0.5, self.on_timer)
        # yaw 재탐지 타이머(메시지가 오래 안 오면 타입 재결정)
        self.yaw_watch = self.create_timer(2.0, self._maybe_resub_yaw)

        # 원점 사전 확정
        if self.origin_mode == 'manual':
            self.lat0, self.lon0 = self.origin_lat, self.origin_lon
            self.origin_set = True
            self._project_route_if_ready()
        elif self.origin_mode == 'csv_first' and self.route_ll:
            self.lat0, self.lon0 = self.route_ll[0]
            self.origin_set = True
            self._project_route_if_ready()

        self.get_logger().info(
            f"[viz] frame='{self.frame_id}', pose_topic='{self.pose_topic}', fix_topic='{self.fix_topic}', "
            f"yaw_topic='{self.yaw_topic}', origin_mode='{self.origin_mode}'"
        )

    # --------- yaw 타입/구독 관리 ---------
    def _setup_yaw_subscription(self):
        # 기존 구독 해제
        if self._yaw_sub is not None:
            try:
                self.destroy_subscription(self._yaw_sub)
            except Exception:
                pass
            self._yaw_sub = None
            self._yaw_type = None

        # 강제 타입 우선
        force = self.yaw_force_type
        if force in ("float64", "float32"):
            self._create_yaw_sub(force)
            return

        # 그래프에서 토픽 타입 조사
        types_map = dict(self.get_topic_names_and_types())
        types = types_map.get(self.yaw_topic, [])

        # 우선순위: float64가 있으면 float64, 아니면 float32
        if 'std_msgs/msg/Float64' in types:
            self._create_yaw_sub('float64')
        elif 'std_msgs/msg/Float32' in types:
            self._create_yaw_sub('float32')
        else:
            # 아직 퍼블리셔가 안 떴을 수 있음 → 일단 float64로 만들어두고 대기
            self._create_yaw_sub('float64')
            self.get_logger().warn(
                f"[yaw] {self.yaw_topic} 타입을 그래프에서 못 찾음 → 임시로 Float64로 구독 생성. "
                f"나중에 메시지가 오래 안 오면 재탐지합니다."
            )

        # 둘 다 있으면 경고(혼재는 바람직하지 않음)
        if 'std_msgs/msg/Float64' in types and 'std_msgs/msg/Float32' in types:
            self.get_logger().warn(
                f"[yaw] {self.yaw_topic} 에 Float32/Float64 둘 다 퍼블리시 중입니다. "
                f"가능하면 한 타입으로 통일하세요. (현재는 {self._yaw_type}로 수신)"
            )

    def _create_yaw_sub(self, kind: str):
        if kind == 'float64':
            self._yaw_sub = self.create_subscription(Float64, self.yaw_topic, self.cb_yaw64, qos_profile_sensor_data)
        elif kind == 'float32':
            self._yaw_sub = self.create_subscription(Float32, self.yaw_topic, self.cb_yaw32, qos_profile_sensor_data)
        else:
            raise ValueError("unknown yaw subscriber type")
        self._yaw_type = kind
        self.get_logger().info(f"[yaw] subscribe {self.yaw_topic} as {kind}")

    def _maybe_resub_yaw(self):
        # 5초 이상 yaw 미수신이면 타입 재탐지/재구독 시도
        if (time.time() - self._last_yaw_stamp) < 5.0:
            return
        types_map = dict(self.get_topic_names_and_types())
        types = types_map.get(self.yaw_topic, [])
        target = None
        if 'std_msgs/msg/Float64' in types:
            target = 'float64'
        elif 'std_msgs/msg/Float32' in types:
            target = 'float32'
        if target and target != self._yaw_type:
            self.get_logger().warn(f"[yaw] no data for a while, re-subscribe as {target}")
            self._setup_yaw_subscription()

    # --------- 콜백 ---------
    def cb_fix(self, msg: NavSatFix):
        if not self.origin_set and self.origin_mode == 'fix':
            try:
                self.lat0 = float(msg.latitude); self.lon0 = float(msg.longitude)
                self.origin_set = True
                self.get_logger().info(f"[origin] fixed by GPS: ({self.lat0:.6f},{self.lon0:.6f})")
                self._project_route_if_ready()
            except Exception:
                return

        try:
            lat = float(msg.latitude); lon = float(msg.longitude)
        except Exception:
            return
        self.last_fix_ll = (lat, lon)
        if self.origin_set:
            self.last_fix_xy = latlon_to_xy_m(lat, lon, self.lat0, self.lon0)

    def cb_pose(self, msg: PoseStamped):
        self.cur_pose = msg
        self.pose_frame_id = msg.header.frame_id or ''
        if (self.pose_frame_id != self.frame_id) and (not self.pose_frame_warned):
            self.pose_frame_warned = True
            self.get_logger().warn(
                f"/current_pose frame_id='{self.pose_frame_id}' != viz frame_id='{self.frame_id}'. "
                f"둘 중 하나로 맞추거나 TF 제공 필요. 빠른 해결: "
                f"`--ros-args -p frame_id:={self.pose_frame_id}` 로 이 노드와 RViz Fixed Frame을 '{self.pose_frame_id}'로 맞추세요."
            )

    def _norm_yaw_east_ccw(self, deg_in: float) -> float:
        # 입력 yaw(deg) → 동=0°, CCW(+) 변환 후 [-180,180)
        deg = float(deg_in)
        if not self.yaw_zero_is_east:
            deg = 90.0 - deg     # N0 → E0
        if not self.yaw_positive_ccw:
            deg = -deg           # CW+ → CCW+
        deg += self.yaw_offset_deg
        return wrap_deg180(deg)

    def cb_yaw64(self, msg: Float64):
        self._handle_yaw(float(msg.data), "Float64")

    def cb_yaw32(self, msg: Float32):
        self._handle_yaw(float(msg.data), "Float32")

    def _handle_yaw(self, raw_deg: float, src: str):
        self.last_yaw_deg = self._norm_yaw_east_ccw(raw_deg)
        self._last_yaw_stamp = time.time()
        if not self._yaw_logged_once:
            self.get_logger().info(f"[yaw:{src}] raw={raw_deg:.2f}° -> east/CCW={self.last_yaw_deg:.2f}°")
            self._yaw_logged_once = True

    # --------- 내부 유틸 ---------
    def _load_route_latlon(self, path):
        pts = []
        try:
            with open(os.path.expanduser(os.path.expandvars(path)), 'r', encoding='utf-8') as f:
                for line in f:
                    s = line.strip()
                    if not s or s.startswith('#'): continue
                    parts = s.replace(',', ' ').split()
                    if len(parts) < 2: continue
                    lat = float(parts[0]); lon = float(parts[1])
                    pts.append((lat, lon))
        except Exception as e:
            self.get_logger().error(f"CSV 읽기 실패: {path} | {e}")
        return pts

    def _project_route_if_ready(self):
        if not (self.origin_set and self.route_ll):
            return
        self.route_xy = [latlon_to_xy_m(lat, lon, self.lat0, self.lon0) for (lat, lon) in self.route_ll]
        self.route_xy_ready = True
        self.get_logger().info(
            f"[route] projected {len(self.route_xy)} pts to ENU using origin=({self.lat0:.6f},{self.lon0:.6f})"
        )

    def _build_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # (A) 경로 LINE_STRIP
        if self.route_xy_ready and len(self.route_xy) >= 2:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = "route_bg"
            m.id = 100
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = self.route_width
            m.color.r, m.color.g, m.color.b, m.color.a = self.route_color
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(x), y=float(y), z=0.0) for (x,y) in self.route_xy]
            ma.markers.append(m)

            # 시작/끝점
            for mid, ix, iy, col in [
                (101, self.route_xy[0][0],  self.route_xy[0][1], (0.2, 0.8, 0.2, 1.0)),
                (102, self.route_xy[-1][0], self.route_xy[-1][1], (0.9, 0.3, 0.3, 1.0)),
            ]:
                s = Marker()
                s.header.frame_id = self.frame_id; s.header.stamp = stamp
                s.ns = "route_bg"; s.id = mid
                s.type = Marker.SPHERE; s.action = Marker.ADD
                s.scale.x = s.scale.y = s.scale.z = self.route_width*3.0
                s.color.r, s.color.g, s.color.b, s.color.a = col
                s.pose.position.x = float(ix); s.pose.position.y = float(iy); s.pose.position.z = 0.0
                ma.markers.append(s)

        # (B) GPS 현재 위치
        if self.show_gps_marker and (self.last_fix_xy is not None):
            gx, gy = self.last_fix_xy
            g = Marker()
            g.header.frame_id = self.frame_id
            g.header.stamp = stamp
            g.ns = "gps_now"; g.id = 300
            g.type = Marker.SPHERE; g.action = Marker.ADD
            g.scale.x = g.scale.y = g.scale.z = self.gps_diam
            g.color.r, g.color.g, g.color.b, g.color.a = self.gps_color
            g.pose.position.x = float(gx); g.pose.position.y = float(gy); g.pose.position.z = 0.0
            ma.markers.append(g)

            if self.last_fix_ll is not None:
                lat, lon = self.last_fix_ll
                t = Marker()
                t.header.frame_id = self.frame_id
                t.header.stamp = stamp
                t.ns = "gps_now"; t.id = 301
                t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
                t.scale.z = self.text_h
                t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
                t.pose.position.x = float(gx)
                t.pose.position.y = float(gy)
                t.pose.position.z = 0.6
                t.text = f"GPS\n{lat:.6f}, {lon:.6f}"
                ma.markers.append(t)

        # (C) yaw_deg 화살표 (최근 GPS 위치 기준)
        if (self.last_yaw_deg is not None) and (self.last_fix_xy is not None):
            yaw_rad = math.radians(self.last_yaw_deg)  # 동=0°, CCW+
            dx = self.yaw_arrow_len * math.cos(yaw_rad)
            dy = self.yaw_arrow_len * math.sin(yaw_rad)
            gx, gy = self.last_fix_xy

            arr = Marker()
            arr.header.frame_id = self.frame_id
            arr.header.stamp = stamp
            arr.ns = "yaw_deg"; arr.id = 400
            arr.type = Marker.ARROW; arr.action = Marker.ADD
            arr.color.r, arr.color.g, arr.color.b, arr.color.a = self.yaw_color
            arr.scale.x = self.arrow_thick
            arr.scale.y = self.arrow_thick * 1.8
            arr.scale.z = self.yaw_arrow_len * 0.3
            arr.points = [Point(x=float(gx), y=float(gy), z=0.0),
                          Point(x=float(gx+dx), y=float(gy+dy), z=0.0)]
            ma.markers.append(arr)

            txt = Marker()
            txt.header.frame_id = self.frame_id
            txt.header.stamp = stamp
            txt.ns = "yaw_deg"; txt.id = 401
            txt.type = Marker.TEXT_VIEW_FACING; txt.action = Marker.ADD
            txt.scale.z = self.text_h
            txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 0.2; txt.color.a = 1.0
            txt.pose.position.x = float(gx)
            txt.pose.position.y = float(gy)
            txt.pose.position.z = 0.9
            txt.text = f"yaw_deg {self.last_yaw_deg:.1f}°"
            ma.markers.append(txt)

        # (D) /current_pose 화살표 (옵션)
        if self.show_pose_marker and (self.cur_pose is not None):
            p = self.cur_pose.pose.position
            q = self.cur_pose.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            dx, dy = self.arrow_len*math.cos(yaw), self.arrow_len*math.sin(yaw)

            arr = Marker()
            arr.header.frame_id = self.frame_id
            arr.header.stamp = stamp
            arr.ns = "pose"; arr.id = 200
            arr.type = Marker.ARROW; arr.action = Marker.ADD
            arr.color.r, arr.color.g, arr.color.b, arr.color.a = 0.2, 0.5, 1.0, 1.0
            arr.scale.x = self.arrow_thick
            arr.scale.y = self.arrow_thick * 1.8
            arr.scale.z = self.arrow_len * 0.3
            arr.points = [Point(x=float(p.x), y=float(p.y), z=float(p.z)),
                          Point(x=float(p.x+dx), y=float(p.y+dy), z=float(p.z))]
            ma.markers.append(arr)

            txt = Marker()
            txt.header.frame_id = self.frame_id
            txt.header.stamp = stamp
            txt.ns = "pose"; txt.id = 201
            txt.type = Marker.TEXT_VIEW_FACING; txt.action = Marker.ADD
            txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 1.0
            txt.scale.z = self.text_h
            txt.pose.position.x = float(p.x)
            txt.pose.position.y = float(p.y)
            txt.pose.position.z = float(p.z) + 0.6
            txt.text = f"pose yaw {math.degrees(yaw):.1f}°"
            ma.markers.append(txt)

        return ma

    # --------- 타이머 ---------
    def on_timer(self):
        if (not self.route_xy_ready) and self.origin_set:
            self._project_route_if_ready()
        ma = self._build_markers()
        self.pub_markers.publish(ma)

def main():
    rclpy.init()
    node = None
    try:
        node = RouteBgAndPoseViz()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()