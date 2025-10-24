#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32, Int32
import numpy as np

# --------------- Variable Setting ---------------
LOOKAHEAD_DISTANCE = 2.8          # 전진 기본 lookahead [m]
REVERSE_LOOKAHEAD_DISTANCE = 0.8   # 후진 전용 lookahead [m] (권장: 0.5~1.0)
MIN_LOOKAHEAD = 0.2                # 최소 lookahead [m]
WHEELBASE = 0.7                    # 차량 휠베이스 [m]

GLOBAL_PATH_TOPIC = '/global_path'
CURRENT_XY_TOPIC = '/current_xy'
CURRENT_YAW_TOPIC = '/current_yaw'
CMD_VEL_TOPIC = '/cmd_vel'
DRIVE_THROTTLE_TOPIC = '/drive_throttle'  # +: 전진, -: 후진, 0: 정지

FORWARD_SPEED = 30.0               # [m/s] 실제 차량에 맞게 조정 필요
REVERSE_SPEED = 5.0                # [m/s] 후진은 안전하게 더 작게

# [ADD] 끝점 근처 헤딩 정렬(옵션)
END_ALIGN_RADIUS = 1.5             # [m] 끝점 반경 안이면 헤딩 정렬 모드로 전환
K_HEADING = 0.8                    # 헤딩 비례 게인
MAX_STEER_AT_END = math.radians(20) # 끝에서 과도 조향 제한 각
# -----------------------------------------------

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.lookahead = LOOKAHEAD_DISTANCE
        self.reverse_lookahead = REVERSE_LOOKAHEAD_DISTANCE
        self.min_lookahead = MIN_LOOKAHEAD
        self.wheelbase = WHEELBASE
        self.forward_speed = FORWARD_SPEED
        self.reverse_speed = REVERSE_SPEED

        # 경로 저장 (Nx2 numpy array)
        self.path_xy = np.empty((0, 2), dtype=np.float64)

        # 현재 상태 저장
        self.current_x = None
        self.current_y = None
        self.current_yaw = None  # [rad], 전진 기준 헤딩

        # 주행 모드: +1 전진, -1 후진, 0 정지 (기본 전진)
        self.drive_mode = 1

        # 구독/퍼블리시
        self.create_subscription(Path, GLOBAL_PATH_TOPIC, self.path_callback, 10)
        self.create_subscription(Pose2D, CURRENT_XY_TOPIC, self.xy_callback, 10)
        self.create_subscription(Float32, CURRENT_YAW_TOPIC, self.yaw_callback, 10)
        self.create_subscription(Int32, DRIVE_THROTTLE_TOPIC, self.throttle_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.get_logger().info("Pure Pursuit node initialized (forward + reverse supported, reverse Ld=0.8m).")

    # ====== Callbacks ======
    def path_callback(self, msg: Path):
        """글로벌 경로를 Nx2 (x,y) 배열로 저장"""
        if not msg.poses:
            self.path_xy = np.empty((0, 2))
            return

        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        self.path_xy = np.column_stack([xs, ys]).astype(np.float64)

    def xy_callback(self, msg: Pose2D):
        """현재 UTM 좌표 업데이트"""
        self.current_x = msg.x
        self.current_y = msg.y
        self._try_compute()

    def yaw_callback(self, msg: Float32):
        """현재 yaw(deg) → rad로 변환 후 업데이트
        기존 보정: (deg + 90) % 360 사용 (사용자 환경 유지)
        """
        self.current_yaw = math.radians((msg.data + 90.0) % 360.0)
        self._try_compute()

    def throttle_callback(self, msg: Int32):
        """주행 모드 업데이트: + → 전진, - → 후진, 0 → 정지"""
        if msg.data > 0:
            self.drive_mode = 1
            self.get_logger().info("Drive mode: FORWARD")
        elif msg.data < 0:
            self.drive_mode = -1
            self.get_logger().info("Drive mode: REVERSE")
        else:
            self.drive_mode = 0
            self.get_logger().info("Drive mode: STOP")
        self._try_compute()

    # ====== Core Logic ======
    def _try_compute(self):
        """현재 위치 + yaw 둘 다 준비됐을 때 Pure Pursuit 실행"""
        if self.path_xy.shape[0] < 2:
            return
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        # 정지 모드면 즉시 정지 명령
        if self.drive_mode == 0:
            self._publish_cmd(0.0, 0.0)
            return

        # 현재 위치 + yaw 확보
        px, py, yaw = self.current_x, self.current_y, self.current_yaw

        # 주행 모드에 따른 lookahead 선택
        Ld_eff = self.lookahead if self.drive_mode == 1 else self.reverse_lookahead

        # lookahead 목표점 찾기 (전진: 진행, 후진: 역방향)
        goal = self._find_lookahead_point_on_path(
            px, py, Ld_eff, self.min_lookahead, self.drive_mode
        )
        if goal is None:
            self.get_logger().warn("No lookahead point found; publishing stop.")
            self._publish_cmd(0.0, 0.0)
            return

        gx, gy = goal  # 목표점

        # [ADD] 끝점 근처라면 헤딩 정렬로 '평행' 맞추기
        # 마지막 세그먼트의 접선 각(path_yaw)을 기준으로 헤딩 오차를 줄임
        last = self.path_xy[-1]
        prev = self.path_xy[-2]
        path_yaw = math.atan2(last[1] - prev[1], last[0] - prev[0])
        dist_to_last = math.hypot(last[0] - px, last[1] - py)
        if dist_to_last <= END_ALIGN_RADIUS:
            desired = path_yaw if self.drive_mode == 1 else (path_yaw + math.pi)
            err = (desired - yaw + math.pi) % (2 * math.pi) - math.pi
            delta_align = max(-MAX_STEER_AT_END, min(MAX_STEER_AT_END, K_HEADING * err))
            v_cmd = self.forward_speed if self.drive_mode == 1 else -self.reverse_speed
            self.get_logger().info(
                f"[END-ALIGN] d_last={dist_to_last:.2f}m, err={math.degrees(err):.1f}deg → steer={math.degrees(delta_align):.1f}deg"
            )
            self._publish_cmd(v_cmd, delta_align)
            return

        # 차량 좌표계 변환: 차량 좌표계는 x전방, y좌측
        dx = gx - px
        dy = gy - py
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        xL =  cos_y * dx + sin_y * dy   # xL>0 앞, xL<0 뒤
        yL = -sin_y * dx + cos_y * dy   # yL>0 좌, yL<0 우

        # 디버그 로그
        dir_lr = "Left" if yL > 0 else ("Right" if yL < 0 else "Center")
        dir_fb = "Forward" if xL > 0 else ("Backward" if xL < 0 else "Stationary")
        mode_str = "FORWARD" if self.drive_mode == 1 else "REVERSE"
        self.get_logger().info(
            f"[{mode_str}] Ld_eff={Ld_eff:.2f} | xL={xL:.3f}, yL={yL:.3f} | Move: {dir_fb}, Steer target side: {dir_lr}"
        )

        # 목표점까지 거리
        Ld = math.hypot(xL, yL)
        if Ld < 1e-6:
            self._publish_cmd(0.0, 0.0)
            return

        # Pure Pursuit 조향각 δ (전진 기준 공식을 후진에도 동일 적용: δ 반전 없음)
        delta = math.atan2(2.0 * self.wheelbase * yL, Ld * Ld)

        # 속도 명령: 전진은 +, 후진은 -
        v_cmd = self.forward_speed if self.drive_mode == 1 else -self.reverse_speed

        # 제어 명령 퍼블리시 (angular.z는 '조향각(rad)'로 사용 중)
        self._publish_cmd(v_cmd, delta)

    # ====== Helpers ======
    def _publish_cmd(self, v: float, steer: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(steer)
        self.cmd_pub.publish(cmd)

    def _find_lookahead_point_on_path(self, px: float, py: float,
                                      Ld: float, min_lookahead: float,
                                      mode: int):
        """가장 가까운 점부터 시작해 lookahead 이상 떨어진 점 반환
        mode=+1: 경로 진행 방향(인덱스 증가), mode=-1: 역방향(인덱스 감소)
        """
        pts = self.path_xy
        if pts.shape[0] < 2:
            return None

        # 1) 가장 가까운 인덱스
        d2 = (pts[:, 0] - px)**2 + (pts[:, 1] - py)**2
        i_near = int(np.argmin(d2))
        target_dist = max(Ld, min_lookahead)

        # 2) 진행/역진 누적거리
        acc = 0.0
        for i in range(i_near, pts.shape[0] - 1):
            seg = pts[i + 1] - pts[i]
            acc += float(np.hypot(seg[0], seg[1]))
            if acc >= target_dist:
                return tuple(pts[i + 1])

        # 3) [ADD] 끝까지 와도 target_dist 못 채우면 → "마지막 세그먼트 방향"으로 가상점 반환
        last = pts[-1]
        prev = pts[-2]
        v = last - prev
        seg_len = float(np.hypot(v[0], v[1]))
        if seg_len < 1e-6:
            return tuple(last)  # 안전장치

        u = v / seg_len
        if mode == -1:
            u = -u  # 후진이면 반대 방향으로 연장

        virtual_goal = last + u * target_dist
        self.get_logger().info(
            f"[VIRTUAL-END] using virtual goal near end: ({virtual_goal[0]:.3f}, {virtual_goal[1]:.3f})"
        )
        return (float(virtual_goal[0]), float(virtual_goal[1]))


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import Pose2D, Twist
# from std_msgs.msg import Float32, Int32
# import numpy as np

# # ----------------- Params -----------------
# LOOKAHEAD_DISTANCE = 2.8
# REVERSE_LOOKAHEAD_DISTANCE = 0.8
# MIN_LOOKAHEAD = 0.2
# WHEELBASE = 0.7

# GLOBAL_PATH_TOPIC = '/global_path'
# CURRENT_XY_TOPIC = '/current_xy'
# CURRENT_YAW_TOPIC = '/current_yaw'
# CMD_VEL_TOPIC = '/cmd_vel'
# DRIVE_THROTTLE_TOPIC = '/drive_throttle'

# FORWARD_SPEED = 30.0
# REVERSE_SPEED = 5.0

# # 끝점 근처 헤딩 정렬(선택적 권장)
# ALIGN_HEADING_AT_END = True
# END_ALIGN_RADIUS = 1.5          # [m]
# K_HEADING = 0.8
# MAX_STEER_AT_END = math.radians(20)

# # 시작부 가상점 사용 조건
# START_ALIGN_RADIUS = 2.0        # [m] 시작점 근처라면 첫 세그먼트 방향 가상점 사용

# # 경로 교체 직후 조향 디바운스
# DEBOUNCE_STEER_SEC = 0.5        # [s] 경로 바뀐 뒤 이 시간 동안 steer=0 고정
# # ------------------------------------------

# class PurePursuit(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_node')

#         self.lookahead = LOOKAHEAD_DISTANCE
#         self.reverse_lookahead = REVERSE_LOOKAHEAD_DISTANCE
#         self.min_lookahead = MIN_LOOKAHEAD
#         self.wheelbase = WHEELBASE
#         self.forward_speed = FORWARD_SPEED
#         self.reverse_speed = REVERSE_SPEED

#         self.path_xy = np.empty((0, 2), dtype=np.float64)

#         self.current_x = None
#         self.current_y = None
#         self.current_yaw = None

#         self.drive_mode = 1

#         self.create_subscription(Path, GLOBAL_PATH_TOPIC, self.path_callback, 10)
#         self.create_subscription(Pose2D, CURRENT_XY_TOPIC, self.xy_callback, 10)
#         self.create_subscription(Float32, CURRENT_YAW_TOPIC, self.yaw_callback, 10)
#         self.create_subscription(Int32, DRIVE_THROTTLE_TOPIC, self.throttle_callback, 10)
#         self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

#         # 경로 교체 디바운스 타임스탬프
#         self.last_path_change = 0.0

#         self.get_logger().info("Pure Pursuit initialized (virtual goals + debounce).")

#     # ===== Callbacks =====
#     def path_callback(self, msg: Path):
#         if not msg.poses:
#             self.path_xy = np.empty((0, 2))
#         else:
#             xs = [p.pose.position.x for p in msg.poses]
#             ys = [p.pose.position.y for p in msg.poses]
#             self.path_xy = np.column_stack([xs, ys]).astype(np.float64)

#         # 경로가 바뀐 시각 기록(디바운스용)
#         self.last_path_change = self.get_clock().now().nanoseconds * 1e-9

#     def xy_callback(self, msg: Pose2D):
#         self.current_x = msg.x
#         self.current_y = msg.y
#         self._try_compute()

#     def yaw_callback(self, msg: Float32):
#         # 기존 보정 유지: (deg + 90) % 360
#         self.current_yaw = math.radians((msg.data + 90.0) % 360.0)
#         self._try_compute()

#     def throttle_callback(self, msg: Int32):
#         self.drive_mode = 1 if msg.data > 0 else (-1 if msg.data < 0 else 0)
#         self._try_compute()

#     # ===== Core =====
#     def _try_compute(self):
#         if self.path_xy.shape[0] < 2:
#             return
#         if self.current_x is None or self.current_y is None or self.current_yaw is None:
#             return

#         # 정지 모드면 즉시 정지
#         if self.drive_mode == 0:
#             self._publish_cmd(0.0, 0.0)
#             return

#         px, py, yaw = self.current_x, self.current_y, self.current_yaw
#         Ld_eff = self.lookahead if self.drive_mode == 1 else self.reverse_lookahead

#         # 경로 교체 직후 조향 디바운스: 짧게 steer=0 유지
#         now = self.get_clock().now().nanoseconds * 1e-9
#         if (now - self.last_path_change) < DEBOUNCE_STEER_SEC:
#             v_cmd = self.forward_speed if self.drive_mode == 1 else -self.reverse_speed
#             self._publish_cmd(v_cmd, 0.0)
#             return

#         # 목표점(가상점 포함) 계산
#         goal = self._find_lookahead_point_on_path(px, py, Ld_eff, self.min_lookahead, self.drive_mode)
#         if goal is None:
#             self._publish_cmd(0.0, 0.0)
#             return
#         gx, gy = goal

#         # 끝점 근처 헤딩 정렬(선택)
#         if ALIGN_HEADING_AT_END and self.path_xy.shape[0] >= 2:
#             last = self.path_xy[-1]
#             prev = self.path_xy[-2]
#             path_yaw = math.atan2(last[1] - prev[1], last[0] - prev[0])
#             dist_to_last = math.hypot(last[0] - px, last[1] - py)
#             if dist_to_last <= END_ALIGN_RADIUS:
#                 desired = path_yaw if self.drive_mode == 1 else (path_yaw + math.pi)
#                 err = (desired - yaw + math.pi) % (2 * math.pi) - math.pi
#                 delta_align = max(-MAX_STEER_AT_END, min(MAX_STEER_AT_END, K_HEADING * err))
#                 v_cmd = self.forward_speed if self.drive_mode == 1 else -self.reverse_speed
#                 self._publish_cmd(v_cmd, delta_align)
#                 return

#         # 차량 좌표계 변환
#         dx, dy = (gx - px), (gy - py)
#         cos_y, sin_y = math.cos(yaw), math.sin(yaw)
#         xL =  cos_y * dx + sin_y * dy
#         yL = -sin_y * dx + cos_y * dy

#         Ld = math.hypot(xL, yL)
#         if Ld < 1e-6:
#             self._publish_cmd(0.0, 0.0)
#             return

#         # Pure Pursuit 조향
#         delta = math.atan2(2.0 * self.wheelbase * yL, Ld * Ld)
#         v_cmd = self.forward_speed if self.drive_mode == 1 else -self.reverse_speed
#         self._publish_cmd(v_cmd, delta)

#     # ===== Helpers =====
#     def _publish_cmd(self, v: float, steer: float):
#         cmd = Twist()
#         cmd.linear.x = float(v)
#         cmd.angular.z = float(steer)
#         self.cmd_pub.publish(cmd)

#     def _start_virtual_goal(self, pts: np.ndarray, target_dist: float, mode: int):
#         """시작점 근처에서는 첫 세그먼트 방향으로 target_dist만큼 앞의 가상점."""
#         p0, p1 = pts[0], pts[1]
#         v = p1 - p0
#         L = float(np.hypot(v[0], v[1]))
#         if L < 1e-6:
#             return tuple(p0)
#         u = v / L
#         if mode == -1:
#             u = -u
#         vg = p0 + u * target_dist
#         return (float(vg[0]), float(vg[1]))

#     def _end_virtual_goal(self, pts: np.ndarray, target_dist: float, mode: int):
#         """끝점 근처에서는 마지막 세그먼트 방향으로 target_dist만큼 앞의 가상점."""
#         last, prev = pts[-1], pts[-2]
#         v = last - prev
#         L = float(np.hypot(v[0], v[1]))
#         if L < 1e-6:
#             return tuple(last)
#         u = v / L
#         if mode == -1:
#             u = -u
#         vg = last + u * target_dist
#         return (float(vg[0]), float(vg[1]))

#     def _find_lookahead_point_on_path(self, px: float, py: float,
#                                       Ld: float, min_lookahead: float,
#                                       mode: int):
#         """
#         - 기본: 가장 가까운 점에서 경로를 따라 target_dist 이상 떨어진 다음 점.
#         - 시작점 근처면: 첫 세그먼트 방향 가상점(급조향 방지).
#         - 끝까지 못 찾으면: 마지막 세그먼트 방향 가상점(평행 정지).
#         """
#         pts = self.path_xy
#         if pts.shape[0] < 2:
#             return None

#         target_dist = max(Ld, min_lookahead)

#         # 가장 가까운 인덱스
#         d2 = (pts[:, 0] - px)**2 + (pts[:, 1] - py)**2
#         i_near = int(np.argmin(d2))

#         # 시작점 근처면 시작 가상점 사용
#         dist_to_start = math.hypot(pts[0, 0] - px, pts[0, 1] - py)
#         if i_near <= 1 and dist_to_start <= START_ALIGN_RADIUS:
#             return self._start_virtual_goal(pts, target_dist, mode)

#         # 진행 방향 누적 거리로 표준 lookahead
#         acc = 0.0
#         for i in range(i_near, pts.shape[0] - 1):
#             seg = pts[i + 1] - pts[i]
#             acc += float(np.hypot(seg[0], seg[1]))
#             if acc >= target_dist:
#                 return tuple(pts[i + 1])

#         # 끝까지 와도 target_dist 못 채운 경우 → 끝 가상점
#         return self._end_virtual_goal(pts, target_dist, mode)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuit()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()