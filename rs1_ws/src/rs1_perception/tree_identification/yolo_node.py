import rclpy
from rclpy.node import Node

from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from rs1_msgs.msg import BoundingBox, YoloDetections

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO('')

        self.yolo_detections = YoloDetections()

        self.parrot_cam_sub = self.create_subscription(
            Image,
            '/parrot1/camera ',
            self.front_camera_callback,
            10
        )
        self.parrot_cam_sub

        self.yolo_img_pub = self.create_publisher(Image, "/yolo/img",1)
        self.yolo_detections_pub = self.create_publisher(YoloDetections, "/yolo/detections",1)

def main (args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()