#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, threading, math
import rclpy
from rclpy.node import Node
import serial
from serial.tools import list_ports
from geometry_msgs.msg import Twist        # 조향 입력(angular.z)
from std_msgs.msg import Int32             # 스로틀 퍼센트 입력(-100..100)

# ================== 프로토콜 템플릿(원형 유지) ==================
forward_cmd = bytes([0x02, 0x34, 0x34, 0x32, 0x33, 0x38, 0x31, 0x30, 0x31, 0x03, 0x00])
stop_cmd    = bytes([0x02, 0x33, 0x42, 0x33, 0x33, 0x38, 0x31, 0x30, 0x31, 0x03, 0x00])
back_cmd    = bytes([0x02, 0x33, 0x35, 0x44, 0x33, 0x38, 0x31, 0x30, 0x31, 0x03, 0x00])
left_cmd    = bytes([0x02, 0x33, 0x42, 0x33, 0x32, 0x32, 0x35, 0x30, 0x31, 0x03, 0x00])
right_cmd   = bytes([0x02, 0x33, 0x42, 0x33, 0x34, 0x44, 0x33, 0x30, 0x31, 0x03, 0x00])

# 위치: [1:4]=속도(3바이트), [4:7]=조향(3바이트) — ASCII-HEX
SPEED_I0, SPEED_I1 = 1, 4
STEER_I0, STEER_I1 = 4, 7

HEX_CHARS = b"0123456789ABCDEF"

def ascii_hex3_to_code(b3: bytes) -> int:
    def val(c):
        c &= 0xFF
        if 48 <= c <= 57: return c - 48
        u = c & ~32
        if 65 <= u <= 70: return u - 55
        raise ValueError("not hex")
    return (val(b3[0]) << 8) | (val(b3[1]) << 4) | val(b3[2])

def code_to_ascii_hex3(code: int) -> bytes:
    code &= 0xFFF
    return bytes([
        HEX_CHARS[(code >> 8) & 0xF],
        HEX_CHARS[(code >> 4) & 0xF],
        HEX_CHARS[(code >> 0) & 0xF],
    ])

# --- 스로틀 앵커 자동 추출 ---
CODE_FWD_MAX = ascii_hex3_to_code(forward_cmd[SPEED_I0:SPEED_I1])
CODE_NEUTRAL = ascii_hex3_to_code(stop_cmd[SPEED_I0:SPEED_I1])
CODE_REV_MAX = ascii_hex3_to_code(back_cmd[SPEED_I0:SPEED_I1])
S_FWD = (CODE_FWD_MAX - CODE_NEUTRAL) / 100.0
S_REV = (CODE_NEUTRAL - CODE_REV_MAX) / 100.0

def throttle_percent_to_code(p: int) -> int:
    p = max(-100, min(100, int(p)))
    codef = CODE_NEUTRAL + (S_FWD * p if p >= 0 else S_REV * p)
    code = round(codef) & 0xFFF
    if abs(code - CODE_NEUTRAL) <= 2:  # 중립 데드밴드
        code = CODE_NEUTRAL
    return code

# --- 조향 앵커 자동 추출 ---
STEER_NEUTRAL   = ascii_hex3_to_code(stop_cmd[STEER_I0:STEER_I1])
STEER_LEFT_MAX  = ascii_hex3_to_code(left_cmd[STEER_I0:STEER_I1])
STEER_RIGHT_MAX = ascii_hex3_to_code(right_cmd[STEER_I0:STEER_I1])

def steer_percent_to_code(p: int) -> int:
    p = max(-100, min(100, int(p)))
    s_pos = (STEER_RIGHT_MAX - STEER_NEUTRAL) / 100.0
    s_neg = (STEER_NEUTRAL - STEER_LEFT_MAX) / 100.0
    codef = STEER_NEUTRAL + (s_pos * p if p >= 0 else s_neg * p)
    code = round(codef) & 0xFFF
    if abs(code - STEER_NEUTRAL) <= 1:  # 약한 데드밴드
        code = STEER_NEUTRAL
    return code

def build_frame(throttle_p: int, steer_p: int | None = None) -> bytes:
    base = bytearray(
        stop_cmd if throttle_p == 0 else (forward_cmd if throttle_p > 0 else back_cmd)
    )
    # 속도
    tcode = throttle_percent_to_code(throttle_p)
    base[SPEED_I0:SPEED_I1] = code_to_ascii_hex3(tcode)
    # 조향
    if steer_p is not None:
        scode = steer_percent_to_code(steer_p)
        base[STEER_I0:STEER_I1] = code_to_ascii_hex3(scode)
    return bytes(base)

def clamp_i(x: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(x))))

# ================== ROS2 노드 ==================
DEFAULT_BAUD = 9600
OPEN_DELAY   = 2.0

class TwistSerialBridge(Node):
    """
    /cmd_vel 구독 → 조향(angular.z)만 사용
    /drive_throttle(Int32, -100..100) 구독 → 스로틀 직접 제어
    - deadman_timeout_s 동안 /cmd_vel 미수신 시: 스로틀/조향 즉시 0
    """

    def __init__(self):
        super().__init__("control_node")

        # ---- 파라미터 ----
        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("interval_ms", 100)

        self.declare_parameter("twist_topic", "/cmd_vel")
        self.declare_parameter("max_speed_mps", 2.0)       # (참고) 실제속도 추정용, 스로틀 매핑에는 직접 사용 안함
        self.declare_parameter("max_steer_rad", 0.35)      # |δ|=max → |steer|=100%
        self.declare_parameter("wheelbase_m", 0.7)         # 요레이트→조향각 환산용
        self.declare_parameter("cmd_is_steer_angle", True) # True: angular.z=조향각(rad), False: 요레이트(rad/s)
        self.declare_parameter("invert_steer", False)
        self.declare_parameter("deadman_timeout_s", 0.5)   # 이 시간 이상 /cmd_vel 미수신 → 정지

        # 스로틀 외부 제어
        self.declare_parameter("throttle_topic", "/drive_throttle")
        self.declare_parameter("throttle_default_pct", 0)  # 시작 스로틀(%): 0=정지

        # ---- 파라미터 로드 ----
        self.port           = self.get_parameter("port").get_parameter_value().string_value
        self.baud           = int(self.get_parameter("baud").value)
        self.interval_ms    = int(self.get_parameter("interval_ms").value)

        self.twist_topic    = self.get_parameter("twist_topic").get_parameter_value().string_value
        self.max_speed_mps  = float(self.get_parameter("max_speed_mps").value)
        self.max_steer_rad  = float(self.get_parameter("max_steer_rad").value)
        self.wheelbase_m    = float(self.get_parameter("wheelbase_m").value)
        self.cmd_is_steer_angle = bool(self.get_parameter("cmd_is_steer_angle").value)
        self.invert_steer   = bool(self.get_parameter("invert_steer").value)
        self.deadman_timeout_s = float(self.get_parameter("deadman_timeout_s").value)

        self.throttle_topic = self.get_parameter("throttle_topic").get_parameter_value().string_value
        self.cur_throt_pct  = int(self.get_parameter("throttle_default_pct").value)  # -100..100, 초기 0

        # ---- 상태 ----
        self._lock = threading.Lock()
        self._have_cmd = False
        self.cur_steer_pct = 0     # -100..100
        self._last_rx_time = time.time()

        # ---- 시리얼 오픈 ----
        self.ser = self._open_serial(self.port, self.baud)

        # ---- 구독 ----
        self.create_subscription(Twist, self.twist_topic, self.cb_twist, 10)
        self.create_subscription(Int32, self.throttle_topic, self.cb_throttle, 1)

        # ---- 타이머(지속 송신) ----
        self.timer = self.create_timer(self.interval_ms / 1000.0, self.on_timer)

        self.get_logger().info(
            f"Started. port={self.port} baud={self.baud} interval_ms={self.interval_ms} "
            f"twist_topic={self.twist_topic} max_steer_rad={self.max_steer_rad} wheelbase_m={self.wheelbase_m} "
            f"deadman_timeout_s={self.deadman_timeout_s} throttle_topic={self.throttle_topic} "
            f"throttle_default_pct={self.cur_throt_pct}"
        )

    # ---- 콜백: 조향 ----
    def cb_twist(self, msg: Twist):
        az = float(msg.angular.z)

        if self.cmd_is_steer_angle:
            # angular.z 를 조향각(rad)로 해석
            steer_pct = clamp_i(100.0 * (az / self.max_steer_rad), -100, 100)
        else:
            # angular.z 를 요레이트(rad/s)로 해석 → δ = atan(r * L / v)
            # v는 현재 스로틀에 비례한 추정속도 사용
            v_est = abs(self.cur_throt_pct) * 0.01 * self.max_speed_mps
            if v_est < 1e-6:
                delta = 0.0
            else:
                delta = math.atan((az * self.wheelbase_m) / v_est)
            steer_pct = clamp_i(100.0 * (delta / self.max_steer_rad), -100, 100)

        if self.invert_steer:
            steer_pct = -steer_pct

        with self._lock:
            self.cur_steer_pct = steer_pct
            self._have_cmd = True
            self._last_rx_time = time.time()

    # ---- 콜백: 스로틀(퍼센트) ----
    def cb_throttle(self, msg: Int32):
        pct = clamp_i(float(msg.data), -100, 100)
        with self._lock:
            self.cur_throt_pct = pct
        # self.get_logger().info(f"[Throttle] set to {pct}%")

    # ---- 타이머: 주기적 전송 ----
    def on_timer(self):
        now = time.time()
        with self._lock:
            timeout = (self.deadman_timeout_s > 0) and ((now - self._last_rx_time) > self.deadman_timeout_s)
            if timeout:
                # /cmd_vel 끊기면 안전 정지
                steer = 0
                throt = 0
            else:
                steer = self.cur_steer_pct if self._have_cmd else 0
                throt = self.cur_throt_pct

        frame = build_frame(throt, -steer)  # 하드웨어 방향 특성(-) 유지
        try:
            self.ser.write(frame)
            self.ser.flush()
            # 디버그 필요시:
            # self.get_logger().info(f"TX throttle={throt}% steer={steer}%")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ---- 유틸 ----
    def _open_serial(self, port: str, baud: int):
        if not port:
            ports = list(list_ports.comports())
            if not ports:
                raise RuntimeError("시리얼 장치를 찾지 못했습니다. 'port' 파라미터를 지정하세요.")
            def score(p):
                desc = (p.description or "").lower()
                dev  = (p.device or "").lower()
                s = 0
                if "arduino" in desc or "ch340" in desc: s += 3
                if "ttyacm" in dev: s += 2
                if "ttyusb" in dev: s += 1
                return s
            ports.sort(key=score, reverse=True)
            port = ports[0].device
            self.get_logger().warning(f"'port' 미지정. 자동 선택: {port} ({ports[0].description})")

        ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        time.sleep(OPEN_DELAY)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        self.get_logger().info(f"[Serial] Opened {port} @ {baud}")
        self.port = port
        return ser

    def destroy_node(self):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("[Serial] Closed.")
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = TwistSerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
