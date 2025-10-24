import os, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

#--------------- Variable Setting ---------------
# CSV 파일 경로 (위도/경도 waypoint 저장)
CSV_FILE = '~/IMU/gps_waypoints.csv'

# 구독할 GPS 토픽 이름
GPS_TOPIC_NAME = '/ublox_gps_node/fix'

# 저장 파일 형식 (space-separated)
# 위도 경도
FILE_FORMAT = "space"  

#-----------------------------------------------

class LatLonLogger(Node):
    def __init__(self):
        super().__init__('gps_latlon_logger')

        # 변수 불러오기
        self.topic = GPS_TOPIC_NAME
        self.out_csv = os.path.expanduser(os.path.expandvars(CSV_FILE))

        # 디렉토리 생성
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)

        # 파일 열기 (line-buffered)
        self.f = open(self.out_csv, 'a', buffering=1)

        # 구독 시작
        self.sub = self.create_subscription(NavSatFix, self.topic, self.cb_fix, qos_profile_sensor_data)
        self.get_logger().info(f"Logging lat lon from '{self.topic}' -> '{self.out_csv}'")

    def cb_fix(self, msg: NavSatFix):
        line = f"{float(msg.latitude):.9f} {float(msg.longitude):.9f}\n"
        self.f.write(line)

    def destroy_node(self):
        try:
            self.f.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = LatLonLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
