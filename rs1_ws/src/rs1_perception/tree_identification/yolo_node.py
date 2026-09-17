import rclpy
from rclpy.node import Node

from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from rs1_msgs.msg import BoundingBox, YoloDetections

bridge = CvBridge()

class YoloNode(Node):
    def __init__(self):
        self.declare_parameter(
            'image_topic',
            '/parrot1/camera/image'
        )

        self.declare_parameter(
            'model_path',
            'best.pt'
        )

        self.declare_parameter(
            'confidence_threshold',
            0.5
        )

        image_topic = self.get_parameter('image_topic').value
        yolo_weights_path = self.get_parameter('model_path').value

        super().__init__('yolo_node')
        self.model = YOLO(yolo_weights_path)

        self.yolo_detections = YoloDetections()

        self.parrot_cam_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.latest_image = None

        # TODO service call for detection /yolo/detect
        self.detect_service = self.create_service(DetectYolo)

        self.yolo_img_pub = self.create_publisher(Image, "/yolo/img",1)
        # self.yolo_detections_pub = self.create_publisher(YoloDetections, "/yolo/detections",1)

    def image_callback(self, msg):
        image = bridge.imgmsg_to_cv2(input,"bgr8")
        self.latest_image = image

    def detect_callback(self, request, response):
        image = self.latest_image

        yolo_detections = YoloDetections()
        yolo_detections.header.frame_id = "yolo_detections"
        yolo_detections.header.stamp = self.get_clock().now().to_msg()

        output = self.model(image)

        for i in output:
            boxes = i.boxes
            for box in boxes:
                bounding_box = BoundingBox()
                coords = box.xyxy[0].to('cpu').detach().numpy().copy()
                box_class = box.cls
                bounding_box.class_name = self.model.names[int(box_class)]
                bounding_box.top = int(coords[0])
                bounding_box.left = int(coords[1])
                bounding_box.bottom = int(coords[2])
                bounding_box.right = int(coords[3])

                yolo_detections.bounding_boxes.append(bounding_box)

        annotated_frame = output[0].plot()
        image_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.yolo_img_pub.publish(image_msg)

        # self.yolo_detections_pub(yolo_detections)
        response.detections = yolo_detections
        ## convert to request/response - currently was just previous publish i had at some point so will probs have it so that response is boxes and/or image based on request.

        return response


def main (args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()