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
        return tuple(pts[-1])


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
