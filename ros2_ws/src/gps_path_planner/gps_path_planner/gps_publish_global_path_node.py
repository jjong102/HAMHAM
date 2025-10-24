#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import utm  # pip install utm
from scipy.interpolate import splprep, splev
import numpy as np

#--------------- Variable Setting ---------------
CSV_FILE = '~/IMU/second/gps_latlon2.csv'  # 초기 파일
SMOOTHING = 0
PUB_TOPIC_NAME = '/global_path'
FRAME_ID = 'map'
#----------------------------------------------

class GPSGlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('gps_global_path_publisher')

        self.csv_file = os.path.expanduser(CSV_FILE)
        self.smoothing = SMOOTHING
        self.pub_topic = PUB_TOPIC_NAME
        self.frame_id = FRAME_ID

        # pub/sub
        self.path_pub = self.create_publisher(Path, self.pub_topic, 10)
        self.sub_set = self.create_subscription(String, '/set_path_file', self.cb_set_path, 10)

        # Path 메시지
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        # 초기 로드
        self.reload_path_from_csv(self.csv_file)

        # 1Hz publish
        self.timer = self.create_timer(1.0, self.publish_path)
        self.get_logger().info(f"Global path publisher initialized with {len(self.path_msg.poses)} points.")

    # ----- runtime switch -----
    def cb_set_path(self, msg: String):
        path = os.path.expanduser((msg.data or '').strip())
        if not path:
            self.get_logger().warn("Empty path for /set_path_file")
            return
        self.reload_path_from_csv(path)

    # ----- loaders -----
    def reload_path_from_csv(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"CSV file not found: {path}")
            return
        self.csv_file = path
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.create_path_from_csv()
        self.get_logger().info(f"Switched path file -> {self.csv_file} (points={len(self.path_msg.poses)})")

    def load_waypoints(self):
        waypoints = []
        if not os.path.exists(self.csv_file):
            self.get_logger().error(f"CSV file not found: {self.csv_file}")
            return waypoints
        with open(self.csv_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.lower().startswith(('latitude','lat','lon')):
                    continue
                parts = line.split()
                if len(parts) < 2:
                    continue
                try:
                    lat = float(parts[0]); lon = float(parts[1])
                    waypoints.append((lat, lon))
                except:
                    continue
        return waypoints

    def smooth_path(self, latlons):
        if not self.smoothing or len(latlons) < 3:
            return latlons
        lats, lons = zip(*latlons)
        tck, _ = splprep([lats, lons], s=self.smoothing)
        unew = np.linspace(0, 1, max(200, len(latlons)*10))
        smooth_lats, smooth_lons = splev(unew, tck)
        return list(zip(smooth_lats, smooth_lons))

    def create_path_from_csv(self):
        waypoints = self.load_waypoints()
        if not waypoints:
            self.get_logger().error("No valid waypoints found. Path not created.")
            return
        processed = self.smooth_path(waypoints)
        for lat, lon in processed:
            x, y, _, _ = utm.from_latlon(lat, lon)
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            self.path_msg.poses.append(pose)
        self.get_logger().info(f"Created Path with {len(self.path_msg.poses)} points (smoothing={'enabled' if self.smoothing else 'disabled'}).")

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSGlobalPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
