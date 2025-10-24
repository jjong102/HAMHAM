# gps_latlon_logger.py  (space-separated)
import os, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

class LatLonLogger(Node):
    def __init__(self):
        super().__init__('gps_latlon_logger')
        self.declare_parameter('topic', '/ublox_gps_node/fix')
        self.declare_parameter('out_csv', 'gps_latlon.txt')  # 확장자는 아무거나 ok
        self.topic   = self.get_parameter('topic').get_parameter_value().string_value
        self.out_csv = self.get_parameter('out_csv').get_parameter_value().string_value

        self.out_csv = os.path.expanduser(os.path.expandvars(self.out_csv))
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)
        self.f = open(self.out_csv, 'a', buffering=1)  # line-buffered

        self.sub = self.create_subscription(NavSatFix, self.topic, self.cb_fix, qos_profile_sensor_data)
        self.get_logger().info(f"Logging lat lon (space-separated) from '{self.topic}' -> '{self.out_csv}'")

    def cb_fix(self, msg: NavSatFix):
        line = f"{float(msg.latitude):.9f} {float(msg.longitude):.9f}\n"
        self.f.write(line)

    def destroy_node(self):
        try: self.f.close()
        finally: super().destroy_node()

def main():
    rclpy.init()
    node = LatLonLogger()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
