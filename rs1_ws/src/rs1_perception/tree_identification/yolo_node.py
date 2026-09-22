#!/usr/bin/env python3
"""YOLO tree identification for the RS1 Parrot camera.

Uses /yolo/detect service running YOLO on the latest RGB frame and returns the detections as YoloDetections msg.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from ultralytics import YOLO

from sensor_msgs.msg import Image
from rs1_interfaces.msg import BoundingBox, YoloDetections
from rs1_interfaces.srv import DetectYolo


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('image_topic', '/parrot1/camera/image')
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')

        self.image_topic = self.get_parameter('image_topic').value
        yolo_weights_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        self.model = YOLO(yolo_weights_path)
        self.latest_image = None
        self.latest_image_header = None

        self.parrot_cam_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.detect_service = self.create_service(
            DetectYolo, '/yolo/detect', self.detect_callback
        )

        self.yolo_img_pub = self.create_publisher(Image, self.debug_image_topic, 1)

        self.get_logger().info(
            f"yolo_node ready (image_topic={self.image_topic}, model={yolo_weights_path}, "
            f"conf={self.confidence_threshold})"
        )

    @staticmethod
    def _image_to_bgr(msg):
        if msg.encoding == "bgr8":
            return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ).copy()
        if msg.encoding == "rgb8":
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ).copy()
            return arr[:, :, ::-1].copy()  # RGB -> BGR
        if msg.encoding in ("mono8", "8UC1"):
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width
            ).copy()
            return np.repeat(arr[:, :, None], 3, axis=2)
        return None

    @staticmethod
    def _bgr_to_image(bgr, header):
        arr = np.ascontiguousarray(bgr, dtype=np.uint8)
        msg = Image()
        msg.header = header
        msg.height = int(arr.shape[0])
        msg.width = int(arr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(arr.shape[1] * 3)
        msg.data = arr.tobytes()
        return msg

    def image_callback(self, msg):
        try:
            image = self._image_to_bgr(msg)
        except Exception as ex:
            self.get_logger().warn(f"Failed to decode image: {ex}")
            return
        if image is None:
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return
        self.latest_image = image
        self.latest_image_header = msg.header

    def detect_callback(self, request, response):
        detections = YoloDetections()
        detections.header.frame_id = "parrot1_camera_rgb_optical_frame"
        detections.header.stamp = self.get_clock().now().to_msg()
        response.detections = detections

        if not request.start_yolo:
            self.get_logger().warn("DetectYolo called with start_yolo=false; returning no detections")
            return response

        if self.latest_image is None:
            self.get_logger().warn("No camera image received yet; returning no detections")
            return response

        image = self.latest_image

        output = self.model(image, conf=self.confidence_threshold, verbose=False)

        for result in output:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                # xyxy -> [x1, y1, x2, y2]
                coords = box.xyxy[0].to('cpu').detach().numpy().copy()
                bounding_box = BoundingBox()
                bounding_box.class_name = self.model.names[int(box.cls)]
                bounding_box.score = float(box.conf)
                bounding_box.left = int(coords[0])
                bounding_box.top = int(coords[1])
                bounding_box.right = int(coords[2])
                bounding_box.bottom = int(coords[3])
                detections.bounding_boxes.append(bounding_box)

        annotated_frame = output[0].plot()
        self.yolo_img_pub.publish(self._bgr_to_image(annotated_frame, detections.header))

        self.get_logger().info(f"Detected {len(detections.bounding_boxes)} tree(s)")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()