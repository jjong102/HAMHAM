#!/usr/bin/env python3
# coding: utf-8

import time
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters
        self.declare_parameter('device_index', 0)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_topic', 'image_raw')  # relative
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('show_window', False)

        idx = int(self.get_parameter('device_index').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.show_window = bool(self.get_parameter('show_window').value)

        # Open camera
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera index {idx}')
            raise RuntimeError('Camera open failed')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Publisher
        self.pub_img = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()

        # Timer
        period = 1.0 / float(fps) if fps > 0 else 0.033
        self.timer = self.create_timer(period, self.on_timer)

        # UI
        self.window_name = f'camera_publisher (/dev/video{idx})'
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, width, height)

        self.get_logger().info(
            f'Started camera publisher: /dev/video{idx}, {width}x{height}@{fps}, topic={image_topic}'
        )

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame grab failed')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub_img.publish(msg)

        if self.show_window:
            cv2.imshow(self.window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quit requested by user (q)')
                rclpy.shutdown()

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap:
                self.cap.release()
            if self.show_window:
                try:
                    cv2.destroyWindow(self.window_name)
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

