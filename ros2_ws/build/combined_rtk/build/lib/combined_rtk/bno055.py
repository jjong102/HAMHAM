import rclpy, re, time, math, serial
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int8MultiArray, String

NUM_RE = re.compile(r'^[\s]*[-+]?\d+(\.\d+)?[\s]*$')  # 순수 숫자 라인
CAL_KV_RE = re.compile(r'(sys|gyro|accel|mag)\s*[:=]\s*([0-3])', re.I)

def is_number_line(s: str) -> bool:
    return NUM_RE.match(s) is not None

def parse_cal_status(line: str):
    """라인에서 sys/gyro/accel/mag 0~3 값을 추출. 없으면 None."""
    found = dict((k, None) for k in ['sys', 'gyro', 'accel', 'mag'])
    for k, v in CAL_KV_RE.findall(line):
        found[k.lower()] = int(v)
    if all(v is not None for v in found.values()):
        return [found['sys'], found['gyro'], found['accel'], found['mag']]
    return None

class BNO055SerialBridge(Node):
    def __init__(self):
        super().__init__('bno055_serial_bridge')

        # ---- 파라미터
        self.declare_parameter('port','/dev/serial/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('topic_yaw', 'bno055/yaw_deg')
        self.declare_parameter('topic_cal', 'bno055/calibration')
        self.declare_parameter('topic_ready', 'bno055/is_calibrated')
        self.declare_parameter('topic_debug', 'bno055/serial_debug')
        self.declare_parameter('to_radians', False)       # True면 rad로 발행
        self.declare_parameter('publish_debug', False)     # 원시 라인 디버그 토픽
        self.declare_parameter('arming_delay_ms', 300)     # 완료 후 발행 지연(ms) - 초기 튀는값 방지

        port   = self.get_parameter('port').value
        baud   = self.get_parameter('baud').value
        self.topic_yaw   = self.get_parameter('topic_yaw').value
        self.topic_cal   = self.get_parameter('topic_cal').value
        self.topic_ready = self.get_parameter('topic_ready').value
        self.topic_debug = self.get_parameter('topic_debug').value
        self.to_radians  = self.get_parameter('to_radians').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.arming_delay = self.get_parameter('arming_delay_ms').value / 1000.0

        # ---- publisher
        self.pub_yaw   = self.create_publisher(Float32, self.topic_yaw, 10)
        self.pub_cal   = self.create_publisher(Int8MultiArray, self.topic_cal, 10)
        self.pub_ready = self.create_publisher(Bool, self.topic_ready, 10)
        self.pub_dbg   = self.create_publisher(String, self.topic_debug, 10) if self.publish_debug else None

        # ---- 상태
        self.calibrated = False
        self.time_cal_finished = None

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
            self.get_logger().info(f'Opened serial {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            raise

        self.timer = self.create_timer(0.01, self.loop)  # 100 Hz 폴링

    def loop(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            low = line.lower()

            # (선택) 디버그 원시 라인 퍼블리시
            if self.pub_dbg:
                self.pub_dbg.publish(String(data=line))

            # 1) 완료 신호 감지
            if ('finish calibration' in low) or ('calibration complete!' in low):
                if not self.calibrated:
                    self.calibrated = True
                    self.time_cal_finished = time.time()
                    self.pub_ready.publish(Bool(data=True))
                    self.get_logger().info('Calibration finished → yaw publishing will start after arming delay')
                return  # 완료 신호 라인은 여기서 종료

            # 2) 캘리브 단계: 상태만 발행, yaw는 절대 발행 안 함
            if not self.calibrated:
                vals = parse_cal_status(line)
                if vals is not None:
                    msg = Int8MultiArray()
                    msg.data = vals  # [sys, gyro, accel, mag]
                    self.pub_cal.publish(msg)
                # 다른 쓰레기 라인은 무시
                return

            # 3) 캘 완료 이후: 지연시간 지난 뒤 숫자 라인만 yaw 발행
            if self.time_cal_finished is not None:
                if (time.time() - self.time_cal_finished) < self.arming_delay:
                    return  # 워밍업 지연

            if is_number_line(line):
                val = float(line)
                if self.to_radians:
                    val = math.radians(val)
                self.pub_yaw.publish(Float32(data=val))
            # 숫자가 아니면 무시(잔여 로그)
        except Exception as e:
            self.get_logger().warn(f'serial error: {e}')

def main():
    rclpy.init()
    node = BNO055SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
