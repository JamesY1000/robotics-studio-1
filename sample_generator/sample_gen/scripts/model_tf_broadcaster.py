#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
import rclpy.parameter
import rclpy.qos
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

"""To provide transforms from model -> world frame"""

NODE_NAME = "model_tf_broadcaster_node"
TF_TOPIC_NAME = "/tf_object_models"


class ModelTFBroadcaster(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Initialise subscriber
        self.tf_subscription = self.create_subscription(TFMessage, TF_TOPIC_NAME, self.tf_callback, 10)

        # Initialise tf_broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Initialised")

    def tf_callback(self, msg):
        transform_msg: TransformStamped = msg.transforms

        self.tf_broadcaster.sendTransform(transform_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ModelTFBroadcaster()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
