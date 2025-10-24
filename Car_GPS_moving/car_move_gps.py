#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

EARTH_R = 6378137.0  # WGS84

def wrap_pi(a: float) -> float:
    while a > math.pi:  a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

def latlon_to_xy_m(lat, lon, lat0, lon0):
    # 지역(수 m~수백 m) 근사: equirectangular
    latr = math.radians(lat); lonr = math.radians(lon)
    lat0r = math.radians(lat0); lon0r = math.radians(lon0)
    x = (lonr - lon0r) * math.cos(lat0r) * EARTH_R  # East
    y = (latr - lat0r) * EARTH_R                    # North
    return x, y

class GpsPathFollowerLookahead(Node):
    def __init__(self):
        super().__init__("gps_path_follower_lookahead")

        # ---------- 기본 CSV 경로: 이 파일과 같은 폴더의 gps_latlon.csv ----------
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_csv = os.path.join(script_dir, "gps_latlon.csv")

        # ---------- Params ----------
        self.declare_parameter("fix_topic", "/ublox_gps_node/fix")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("waypoints_csv", default_csv)      # 각 줄: "lat lon"

        # 진행벡터/도달/룩어헤드
        self.declare_parameter("heading_update_min_move_m", 0.30)  # 30cm 이동 시 헤딩 갱신
        self.declare_parameter("wp_reach_min_m", 0.30)
        self.declare_parameter("wp_reach_max_m", 0.60)
        self.declare_parameter("wp_reach_scale", 0.6)              # reach = clamp(k*spacing)
        self.declare_parameter("lookahead_dist_m", 1.0)            # 누적거리 L 앞 점을 목표
        self.declare_parameter("control_rate_hz", 10.0)

        # 조향/필터
        self.declare_parameter("steering_gain", 1.0)               # δ = Kp * Δθ
        self.declare_parameter("steer_lpf_tau_s", 0.6)             # 1차 LPF 시정수
        self.declare_parameter("max_steer_rad", 0.6)               # ≈34°

        # 속도 출력(브릿지 호환 기본: 퍼센트)
        self.declare_parameter("output_linear_is_percent", True)
        self.declare_parameter("slow_speed_pct", 20.0)             # 헤딩 생성 전
        self.declare_parameter("cruise_speed_pct", 35.0)
        self.declare_parameter("max_speed_mps", 2.0)               # m/s 모드일 때

        # 각도 크면 감속(선택)
        self.declare_parameter("angle_slowdown_enable", True)
        self.declare_parameter("angle_slowdown_thresh_rad", 0.6)
        self.declare_parameter("angle_slowdown_pct", 22.0)

        # 전처리(중복/초근접 제거)
        self.declare_parameter("preprocess_min_seg_m", 0.04)       # 4 cm 이하 구간 제거

        # ★ 캐치업 모드(경로에서 멀어지면 가까운 점 기준으로 재타깃)
        self.declare_parameter("catchup_use_nearest_if_far", True)
        self.declare_parameter("catchup_far_dist_m", 3.0)

        # ---------- Load Params ----------
        self.fix_topic   = self.get_parameter("fix_topic").get_parameter_value().string_value
        self.cmd_topic   = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.csv_path    = os.path.expanduser(self.get_parameter("waypoints_csv").get_parameter_value().string_value)

        self.min_move    = float(self.get_parameter("heading_update_min_move_m").value)
        self.reach_min   = float(self.get_parameter("wp_reach_min_m").value)
        self.reach_max   = float(self.get_parameter("wp_reach_max_m").value)
        self.reach_k     = float(self.get_parameter("wp_reach_scale").value)
        self.lookahead   = float(self.get_parameter("lookahead_dist_m").value)
        self.rate_hz     = float(self.get_parameter("control_rate_hz").value)

        self.kp          = float(self.get_parameter("steering_gain").value)
        self.tau         = float(self.get_parameter("steer_lpf_tau_s").value)
        self.max_delta   = float(self.get_parameter("max_steer_rad").value)

        self.out_percent = bool(self.get_parameter("output_linear_is_percent").value)
        self.v_slow_pct  = float(self.get_parameter("slow_speed_pct").value)
        self.v_cruise_pct= float(self.get_parameter("cruise_speed_pct").value)
        self.v_max_mps   = float(self.get_parameter("max_speed_mps").value)

        self.angle_slow  = bool(self.get_parameter("angle_slowdown_enable").value)
        self.angle_th    = float(self.get_parameter("angle_slowdown_thresh_rad").value)
        self.angle_vpct  = float(self.get_parameter("angle_slowdown_pct").value)

        self.min_seg_m   = float(self.get_parameter("preprocess_min_seg_m").value)
        self.catchup_nearest = bool(self.get_parameter("catchup_use_nearest_if_far").value)
        self.catchup_far     = float(self.get_parameter("catchup_far_dist_m").value)

        # ---------- State ----------
        self.cur_lat = None; self.cur_lon = None
        self.lat0 = None; self.lon0 = None; self.origin_set = False
        self.cur_x = 0.0; self.cur_y = 0.0

        self.heading_valid = False
        self.heading_anchor_x = None
        self.heading_anchor_y = None
        self.heading_rad = 0.0

        self.delta_lpf = 0.0
        self.last_ctrl_time = time.time()
        self._last_log_t = 0.0

        self.wps_ll = self._load_waypoints_ll(self.csv_path)   # [(lat,lon), ...]
        self.wps_xy = []                                        # origin 정한 뒤 변환
        self.wp_idx = 0

        # ---------- ROS I/O ----------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub_fix = self.create_subscription(NavSatFix, self.fix_topic, self.cb_fix, qos)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer   = self.create_timer(1.0/self.rate_hz, self.on_timer)

        self.get_logger().info(
            f"Loaded {len(self.wps_ll)} waypoints from {self.csv_path}. lookahead={self.lookahead} m"
        )

    # ---------- Waypoints ----------
    def _load_waypoints_ll(self, path):
        wps = []
        if not os.path.exists(path):
            self.get_logger().error(f"CSV not found: {path}")
            return wps
        with open(path, "r") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                parts = s.split()
                if len(parts) < 2:
                    continue
                wps.append((float(parts[0]), float(parts[1])))
        if not wps:
            self.get_logger().error("No waypoints loaded.")
        return wps

    def _preprocess_wps_xy(self, pts, min_seg_m=0.04):
        """이웃 점 간 거리가 min_seg_m 미만이면 스킵(중복/초근접 제거) + 마지막 점 보존."""
        if not pts:
            return pts
        out = [pts[0]]
        lx, ly = pts[0]
        for (x, y) in pts[1:]:
            if math.hypot(x - lx, y - ly) >= min_seg_m:
                out.append((x, y)); lx, ly = x, y
        # ★ 마지막 점은 무조건 보존
        if out[-1] != pts[-1]:
            out.append(pts[-1])
        return out

    def _nearest_wp_index(self, x, y, min_index=0):
        """
        현 위치 (x,y)에 가장 가까운 웨이포인트 인덱스.
        ★ 앞만 보기: min_index 이상에서만 검색하여 뒤 인덱스는 무시.
        """
        if not self.wps_xy:
            return 0
        start = max(0, int(min_index))
        last = len(self.wps_xy) - 1
        if start > last:
            return last
        return min(range(start, len(self.wps_xy)),
                   key=lambda i: (self.wps_xy[i][0]-x)**2 + (self.wps_xy[i][1]-y)**2)

    def _ensure_origin_and_xy(self):
        if self.origin_set or self.cur_lat is None:
            return
        self.lat0 = self.cur_lat; self.lon0 = self.cur_lon
        self.origin_set = True
        # ENU 변환 후 전처리
        raw_xy = [latlon_to_xy_m(lat, lon, self.lat0, self.lon0) for (lat,lon) in self.wps_ll]
        self.wps_xy = self._preprocess_wps_xy(raw_xy, self.min_seg_m)
        # 초기 헤딩 앵커는 현위치(원점)
        self.heading_anchor_x = 0.0
        self.heading_anchor_y = 0.0
        # ★ 시작 시 가장 가까운 웨이포인트로 스냅
        self.wp_idx = self._nearest_wp_index(0.0, 0.0, 0)
        self.get_logger().info(
            f"Origin fixed at lat={self.lat0:.8f}, lon={self.lon0:.8f} | "
            f"Waypoints after preprocess: {len(self.wps_xy)} | start wp_idx={self.wp_idx}"
        )

    # ---------- Subscriptions ----------
    def cb_fix(self, msg: NavSatFix):
        self.cur_lat = float(msg.latitude)
        self.cur_lon = float(msg.longitude)
        self._ensure_origin_and_xy()
        if self.origin_set:
            self.cur_x, self.cur_y = latlon_to_xy_m(self.cur_lat, self.cur_lon, self.lat0, self.lon0)

    # ---------- Helpers ----------
    def _current_wp_xy(self, idx=None):
        i = self.wp_idx if idx is None else idx
        if i >= len(self.wps_xy):
            return None
        return self.wps_xy[i]

    def _spacing_to_next(self, i):
        if i+1 >= len(self.wps_xy): return None
        x1,y1 = self.wps_xy[i]; x2,y2 = self.wps_xy[i+1]
        return math.hypot(x2-x1, y2-y1)

    def _dynamic_reach(self, i):
        spacing = self._spacing_to_next(i)
        if spacing is None:
            return max(self.reach_min, min(self.reach_max, self.reach_max))
        r = self.reach_k * spacing
        return max(self.reach_min, min(self.reach_max, r))

    def _advance_if_reached_or_passed(self):
        """거리/지나침 판정으로 여러 점 연속 스킵."""
        max_skip = 2000
        k = 0
        while self.wp_idx < len(self.wps_xy):
            wp = self._current_wp_xy()
            if wp is None: break
            wx, wy = wp
            dx = self.cur_x - wx; dy = self.cur_y - wy
            dist = math.hypot(dx, dy)
            reach = self._dynamic_reach(self.wp_idx)

            passed = False
            if self.wp_idx + 1 < len(self.wps_xy):
                nx, ny = self.wps_xy[self.wp_idx+1]
                vx, vy = (nx - wx), (ny - wy)                 # WP→next
                px, py = (self.cur_x - wx), (self.cur_y - wy) # WP→pos
                passed = (vx*px + vy*py) > 0.0

            if (dist < reach) or passed:
                self.wp_idx += 1
                k += 1
                if k >= max_skip: break
                continue
            break

    def _target_index_by_lookahead(self, start_idx=None):
        """
        start_idx부터 누적거리 lookahead 이상 떨어진 인덱스(없으면 마지막).
        ★ 앞만 보기: 항상 self.wp_idx 이상에서만 탐색.
        """
        if not self.wps_xy:
            return 0
        base = self.wp_idx if start_idx is None else max(self.wp_idx, int(start_idx))
        last = len(self.wps_xy) - 1
        if base >= last:
            return last
        acc = 0.0
        i = base
        while i < last and acc < self.lookahead:
            x1,y1 = self.wps_xy[i]
            x2,y2 = self.wps_xy[i+1]
            acc += math.hypot(x2-x1, y2-y1)
            i += 1
        return min(i, last)

    # ---------- Control ----------
    def on_timer(self):
        now = time.time()
        dt = max(1e-3, now - self.last_ctrl_time)
        self.last_ctrl_time = now

        twist = Twist()

        # 1) GPS가 아직 없으면 대기
        if self.cur_lat is None:
            self.pub_cmd.publish(twist)
            return

        # 2) 원점(Origin) 미설정: 초기 저속 직진으로 헤딩 생성 준비
        if not self.origin_set:
            v = self.v_slow_pct
            twist.linear.x = float(v) if self.out_percent else float(v)/100.0*self.v_max_mps
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)
            return

        # 3) 웨이포인트 미존재
        if len(self.wps_xy) == 0:
            self.pub_cmd.publish(twist)
            return

        # 4) 도달/지나침 스킵
        finished_before = (self.wp_idx >= len(self.wps_xy))
        self._advance_if_reached_or_passed()
        if self.wp_idx >= len(self.wps_xy):
            self.pub_cmd.publish(twist)
            if not finished_before:
                self.get_logger().info("Final waypoint reached. Stopping.")
            return

        # 5) 진행벡터(heading) 생성 및 갱신
        move_dx = self.cur_x - (self.heading_anchor_x if self.heading_anchor_x is not None else self.cur_x)
        move_dy = self.cur_y - (self.heading_anchor_y if self.heading_anchor_y is not None else self.cur_y)
        move_dist = math.hypot(move_dx, move_dy)

        if not self.heading_valid:
            v_cmd_pct = self.v_slow_pct
            delta_cmd_raw = 0.0
            if move_dist >= self.min_move:
                self.heading_rad = math.atan2(move_dy, move_dx)
                self.heading_valid = True
                self.heading_anchor_x = self.cur_x
                self.heading_anchor_y = self.cur_y
        else:
            if move_dist >= self.min_move:
                self.heading_rad = math.atan2(move_dy, move_dx)
                self.heading_anchor_x = self.cur_x
                self.heading_anchor_y = self.cur_y

            # --- 타겟 선택 (앞만 보기 캐치업 포함) ---
            cur_wp = self._current_wp_xy()
            if cur_wp is not None:
                dist_to_wp = math.hypot(cur_wp[0]-self.cur_x, cur_wp[1]-self.cur_y)
            else:
                dist_to_wp = float("inf")

            if self.catchup_nearest and dist_to_wp > self.catchup_far:
                # ★ 너무 멀면 '현재 진행 인덱스 이상'에서 가장 가까운 점으로만 캐치업
                near_i = self._nearest_wp_index(self.cur_x, self.cur_y, self.wp_idx)
                base_i = max(self.wp_idx, near_i)
                self.wp_idx = base_i                     # 뒤로 가지 않음
                tgt_idx = self._target_index_by_lookahead(start_idx=base_i)
            else:
                # 일반 룩어헤드(자연스럽게 self.wp_idx 이상에서만 탐색)
                tgt_idx = self._target_index_by_lookahead()

            tx, ty = self.wps_xy[tgt_idx]
            dir_goal = math.atan2(ty - self.cur_y, tx - self.cur_x)

            dtheta = wrap_pi(dir_goal - self.heading_rad)  # 현재 진행방향 대비 목표방향
            delta_cmd_raw = self.kp * dtheta
            delta_cmd_raw = max(-self.max_delta, min(self.max_delta, delta_cmd_raw))

            v_cmd_pct = self.v_cruise_pct
            if self.angle_slow and abs(dtheta) > self.angle_th:
                v_cmd_pct = min(v_cmd_pct, self.angle_vpct)

        # 6) 조향 LPF
        alpha = math.exp(-dt / max(1e-3, self.tau))
        self.delta_lpf = alpha * self.delta_lpf + (1.0 - alpha) * delta_cmd_raw
        delta_cmd = max(-self.max_delta, min(self.max_delta, self.delta_lpf))

        # 7) 출력(브릿지 호환)
        if self.out_percent:
            twist.linear.x = float(v_cmd_pct)             # %로 해석
        else:
            twist.linear.x = float(v_cmd_pct) / 100.0 * self.v_max_mps
        twist.angular.z = float(delta_cmd)                # 조향각(rad)

        self.pub_cmd.publish(twist)

        # 로그(1초 간격)
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            cur_wp = self._current_wp_xy()
            if cur_wp is not None:
                dist_to_wp = math.hypot(cur_wp[0]-self.cur_x, cur_wp[1]-self.cur_y)
            else:
                dist_to_wp = float("inf")
            self.get_logger().info(
                f"wp_idx={self.wp_idx}/{len(self.wps_xy)} dist_wp={dist_to_wp:.2f}m "
                f"heading_ok={self.heading_valid} δ={delta_cmd:.3f}rad v_out="
                + (f"{int(v_cmd_pct)}%" if self.out_percent else f"{twist.linear.x:.2f} m/s")
            )

def main():
    rclpy.init()
    node = None
    try:
        node = GpsPathFollowerLookahead()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
