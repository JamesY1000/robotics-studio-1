#!/usr/bin/env python3

from pathlib import Path
import sys
import select
import termios
import tty

import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
import rclpy.qos
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
import numpy as np
import ros2_numpy as rnp
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, Image

import ultralytics
from ultralytics import YOLO

try:
    from brain3_common import RobotTopics  # type: ignore
except ImportError:
    from sample_gen.services import make_topics_namespace
    RobotTopics = make_topics_namespace()

"""Converts laserscans to image, runs inference, publishes (and broadcasts) detected poses"""

NODE_NAME = "yolo_inference_node"
IMG_SIZE = 1280
MODEL_PATH = "/brain2/docker/deployment/yolov8/tx2/models/yolo11n-obb.pt"
RESULTS_PATH = "/brain2/py3/yolo_backend/inference/results"


class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        print(ultralytics.__file__)  # Ensure this is our custom ultralytics to enable grayscale inference

        # Create merged_pc2 subscriber
        self.laser_sub = self.create_subscription(
            PointCloud2, RobotTopics.LASER_MERGED_PC2.value, self.lasercallback, rclpy.qos.qos_profile_sensor_data
        )

        # Create image publisher
        self.img_pub = self.create_publisher(Image, RobotTopics.YOLO_V8_IMAGE.value, 10)

        # Create pose publisher
        self.pose_pub = self.create_publisher(PoseArray, RobotTopics.YOLO_DETECTED_POSE.value, 10)

        # Create tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        Path(RESULTS_PATH).mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Saved results can be found at {RESULTS_PATH}")

        self.bridge = cv_bridge.CvBridge()

        self.model = YOLO(MODEL_PATH)

        self.image = None

        self.img_count = 0

        self.get_logger().info("Initialised")

    def lasercallback(self, msg: PointCloud2):
        # Convert pc2 to numpy array
        pc = rnp.numpify(msg)
        if pc is not None:
            points = np.zeros((pc.shape[0], 3), dtype=np.float32)
            points[:, 0] = pc["x"]
            points[:, 1] = pc["y"]
            points[:, 2] = pc["z"]

        # Convert pc2 numpy array to image
        image, scale = self.generate_point_cloud_image(points, IMG_SIZE)
        self.image = image
        self.scale = scale

        # Publish img
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
        self.img_pub.publish(image_msg)

        # self.perform_inference() # Uncomment here for inference at every scan callback

    def generate_point_cloud_image(
        self, points: np.ndarray, img_size: int = 1280, bounds: float = 5
    ) -> tuple[np.ndarray, float]:
        """
        Converts a 3D point cloud into a 2D grayscale image.

        :param points: List of (x, y, z) coordinates.
        :param img_size: Size of the output image in pixels (default: 1280x1280).
        :param bounds: Defines the real-world range in meters (default: ±5m).
        :return: NumPy image array of shape (img_size, img_size, 1) and the scale factor.
        """
        # Convert ±bounds meters into img_size pixels
        scale = img_size / (2 * bounds)
        image = np.zeros((img_size, img_size), dtype=np.uint8)  # Single channel for grayscale

        # Convert 3D points to 2D image coordinates
        for x, y, _ in points:
            if -bounds <= x <= bounds and -bounds <= y <= bounds:
                # Shift and scale x and y
                px = int((x + bounds) * scale)
                py = int((y + bounds) * scale)
                # Ensure the pixel coordinates are within the image bounds
                if 0 <= px < img_size and 0 <= py < img_size:
                    # Make white pixel for each point
                    image[py, px] = 255

        return image, scale

    def perform_inference(self):
        """
        On key press, saves current image and runs inference on image.

        :return: None
        """
        # Save image
        if self.image is not None:
            # TODO: (james) Inference directly on grayscale
            # Ensure the image is in the correct format (height, width, channels)
            if len(self.image.shape) == 2:  # If the image is grayscale
                self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2RGB)

            # Run inference directly on the image array
            results = self.model.predict(source=self.image)

            results = self.model.predict(source=self.image)
            detected_poses = []

            for i, result in enumerate(results):
                obbs = result.obb

                if obbs is not None:
                    for idx, obb in enumerate(obbs):
                        # Check confidence score
                        confidence = obb.conf.cpu().numpy()[0]
                        self.get_logger().info(f"Confidence: {confidence}")
                        if confidence >= 0.95:
                            pose = self.calculate_pose(obb, self.scale, 5)  # Assuming bounds is ±5 meters
                            self.get_logger().info(f"Pose: {pose}")
                            detected_poses.append(pose)

                            # Publish the transform from "world" to "detected_<idx>"
                            t = TransformStamped()
                            t.header.stamp = self.get_clock().now().to_msg()
                            t.header.frame_id = "world"
                            t.child_frame_id = f"detected_{idx}"
                            t.transform.translation.x = pose.position.x
                            t.transform.translation.y = pose.position.y
                            t.transform.translation.z = pose.position.z
                            t.transform.rotation = pose.orientation
                            self.tf_broadcaster.sendTransform(t)

                # Publish poses if any detected with sufficient confidence
                if detected_poses:
                    detected_poses_msg = PoseArray()
                    detected_poses_msg.header.frame_id = "world"  # Set the frame_id to the appropriate frame
                    detected_poses_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
                    detected_poses_msg.poses = detected_poses
                    self.pose_pub.publish(detected_poses_msg)

                result.save(filename=f"{RESULTS_PATH}/{self.img_count}.jpg")  # save results
            self.img_count += 1

    def calculate_pose(self, obb, scale, bounds) -> Pose:
        """
        Calculate pose from oriented bounding box.

        :param obb: Oriented bounding box detected by the model.
        :param scale: Scale factor to convert pixel coordinates to real-world coordinates.
        :param bounds: Real-world bounds in meters.
        :return: Pose calculated from OBB.
        """
        pose = Pose()

        # Extract the center coordinates, dimensions, and orientation from the xywhr tensor
        xywhr = obb.xywhr.cpu().numpy()[0]  # Move tensor to CPU and convert to numpy array
        centre_x, centre_y, width, height, rotation = xywhr

        self.get_logger().info(f"x: {centre_x}, y: {centre_y}, w: {width}, h: {height}, r: {rotation}")

        # Convert pixel coordinates to real-world coordinates using the scale factor
        centre_x = (centre_x / scale) - bounds  # Adjust to real-world coordinates
        centre_y = (centre_y / scale) - bounds

        # Set the position in the pose
        pose.position = Point(x=float(centre_x), y=float(centre_y), z=0.0)

        # Convert rotation (yaw) to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0, 0, rotation)  # Roll, Pitch, Yaw

        # Set the orientation in the pose
        pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()

    # rclpy.spin(node) # Uncomment here for inference at every scan callback
    # node.destroy_node()
    # rclpy.shutdown()

    # Set up terminal to read key input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                key = sys.stdin.read(1)
                if key == "k":
                    node.perform_inference()
                elif key == "\x03":  # Ctrl+C
                    break
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
