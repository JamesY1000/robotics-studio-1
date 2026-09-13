import rclpy
from rclpy.node import Node

from ultralytics import YOLO
from cv_bridge import CvBridge

from yolo_msg.msg import BoundingBox, YoloDetections

class YoloNode(Node):
    ...