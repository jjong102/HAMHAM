#!/usr/bin/env python3
# coding: utf-8
"""
ROS2 노드: 카메라 캡처 + YOLOv8 추론 + 콘 상태 String 토픽 퍼블리시 (+ROI 크롭)

클래스(3):
['cone_B', 'cone_Y', 'drum']

퍼블리시(String):
/cone/state  # 최상위(최대 conf) 라벨 또는 "none"
"""

import os
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

from ultralytics import YOLO
try:
    import torch
    _HAS_TORCH = True
except Exception:
    _HAS_TORCH = False

# ── 원하는 화면 영역(크롭) 설정 ──
CROP_CFG = {
    "left": 400,     # 왼쪽에서 제거할 픽셀
    "right": 400,    # 오른쪽에서 제거할 픽셀
    "bottom": 0,   # 아래에서 제거할 픽셀
    "enabled": True, # 크롭 사용 on/off
}

class ConePublisher(Node):
    def __init__(self):
        super().__init__('cone_detector')

        # ───────────────────────── 입력 소스/카메라 파라미터 ─────────────────────────
        # image_in 이 지정되면 해당 이미지 토픽을 구독하고, 비어있으면 카메라 장치를 엽니다.
        self.declare_parameter('image_in', '')
        self.declare_parameter('device_index', 6)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('show_window', True)

        image_in = self.get_parameter('image_in').get_parameter_value().string_value
        idx    = int(self.get_parameter('device_index').value)
        width  = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps    = int(self.get_parameter('fps').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        # ───────────────────────── YOLO 설정 ─────────────────────────
        self.YOLO_CFG = {
            "model_path": "cone_color.pt",
            "class_names": ['cone_B', 'cone_Y', 'drum'],
            "conf": 0.5,
            "iou": 0.45,
            "imgsz": 640,
        }

        self.device = "cuda:0" if (_HAS_TORCH and torch.cuda.is_available()) else "cpu"

        # CvBridge (구독/퍼블리시 공통)
        self.bridge = CvBridge()

        # ───────────────────────── 입력 소스 설정 ─────────────────────────
        self.use_image_sub = bool(image_in)
        if self.use_image_sub:
            # 이미지 구독 모드
            self.sub_img = self.create_subscription(Image, image_in, self.image_cb, 10)
        else:
            # 카메라 오픈 모드
            self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error(f'Cannot open camera index {idx}')
                raise RuntimeError('Camera open failed')

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)

        # ───────────────────────── 퍼블리셔 ─────────────────────────
        self.pub_img = self.create_publisher(Image, '/lane/outline', 10)

        self.pub_state  = self.create_publisher(String, '/cone/state', 10)

        # ───────────────────────── YOLO 로드 ─────────────────────────
        model_path = self.YOLO_CFG["model_path"]
        if not os.path.exists(model_path):
            self.get_logger().error(f"YOLO model not found: {model_path}")
            raise FileNotFoundError(f"Missing model file: {model_path}")

        self.model = YOLO(model_path)
        self.class_names = self.YOLO_CFG["class_names"]
        self.cls_to_name = {i: name for i, name in enumerate(self.class_names)}

        # 시각화 색상
        self.color_map = {
            "cone_B": (255, 0, 0),
            "cone_Y": (0, 255, 255),
            "drum": (128, 128, 128),
        }

        # UI
        self.window_name = 'cone_detector (press q to quit)'
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, width, height)

        # FPS
        self._t_last = time.time()
        self._fps_smooth = 0.0

        # 타이머(카메라 모드에서만 사용)
        if not self.use_image_sub:
            period = 1.0/float(fps) if fps > 0 else 0.03
            self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            (f'Input: image_in={image_in}' if self.use_image_sub else f'Started camera: /dev/video{idx}, {width}x{height}@{fps}, show_window={self.show_window}') + "\n"
            f'YOLOv8: {model_path}, device={self.device}, conf={self.YOLO_CFG["conf"]}, iou={self.YOLO_CFG["iou"]}\n'
            f'Crop: enabled={CROP_CFG["enabled"]} L={CROP_CFG["left"]} R={CROP_CFG["right"]} B={CROP_CFG["bottom"]}'
        )

    # ───────────────────────── ROI 크롭 ─────────────────────────
    def _apply_crop(self, frame):
        if not CROP_CFG.get("enabled", True):
            return frame

        h, w = frame.shape[:2]
        L = max(0, int(CROP_CFG.get("left", 0)))
        R = max(0, int(CROP_CFG.get("right", 0)))
        B = max(0, int(CROP_CFG.get("bottom", 0)))

        x0 = L
        x1 = max(L + 1, w - R)
        y0 = 0
        y1 = max(1, h - B)

        if x0 >= x1 or y0 >= y1:
            self.get_logger().warn(
                f"Invalid crop: w={w}, h={h}, L={L}, R={R}, B={B} → using full frame"
            )
            return frame

        return frame[y0:y1, x0:x1]

    # ───────────────────────── 프레임 처리 공통 ─────────────────────────
    def _handle_frame(self, frame):

        # (A) 크롭
        frame = self._apply_crop(frame)

        # (B) YOLO 추론
        t0 = time.time()
        results = self.model.predict(
            frame,
            imgsz=self.YOLO_CFG["imgsz"],
            conf=self.YOLO_CFG["conf"],
            iou=self.YOLO_CFG["iou"],
            device=self.device,
            verbose=False
        )
        infer_dt = time.time() - t0

        # (C) 최상위 state 선정
        top_label = "none"
        top_conf = -1.0

        if len(results) > 0:
            r0 = results[0]
            boxes = r0.boxes
            if boxes is not None and boxes.xyxy is not None:
                xyxy = boxes.xyxy.cpu().numpy().astype(np.int32)
                conf = boxes.conf.cpu().numpy()
                cls  = boxes.cls.cpu().numpy().astype(int)

                for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                    name = self.cls_to_name.get(k, f"cls{k}")

                    # 최상위 라벨
                    if c > top_conf and name in self.class_names:
                        top_conf = c
                        top_label = name

                    # 시각화
                    color = self.color_map.get(name, (0, 180, 255))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f"{name} {c:.2f}"
                    cv2.putText(frame, label, (x1, max(20, y1-6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

        # (D) OSD
        t = time.time()
        dt = t - self._t_last
        self._t_last = t
        if dt > 0:
            inst = 1.0 / dt
            alpha = 0.1
            self._fps_smooth = (1 - alpha) * self._fps_smooth + alpha * inst if self._fps_smooth else inst

        cv2.putText(frame,
                    f"FPS:{self._fps_smooth:.1f}  INF:{(infer_dt*1000):.1f}ms  STATE:{top_label}  "
                    f"CROP {frame.shape[1]}x{frame.shape[0]}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

        # (E) 이미지 퍼블리시
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.pub_img.publish(msg)

        # (F) String 퍼블리시
        self.pub_state.publish(String(data=top_label))

        # (G) 미리보기
        if self.show_window:
            cv2.imshow(self.window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quit requested by user (q)')
                rclpy.shutdown()

    # ───────────────────────── 카메라 타이머 콜백 ─────────────────────────
    def timer_cb(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame grab failed')
            return
        self._handle_frame(frame)

    # ───────────────────────── 이미지 구독 콜백 ─────────────────────────
    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge conversion failed: {e}')
            return
        self._handle_frame(frame)

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
    node = ConePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
