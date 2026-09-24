#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
import rclpy.parameter
import rclpy.qos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
try:
    from brain3_common.name_defs import ArculeeTopics  # type: ignore
except ImportError:
    from sample_gen.services import make_topics_namespace
    ArculeeTopics = make_topics_namespace()


"""To provide transform from arculee -> world frame"""

NODE_NAME = "arculee_world_broadcaster_node"


class ArculeeWorldBroadcaster(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Initialise static tf_static_broadcaster odom->world
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.broadcast_odom_to_world()

        # Initialise tf_broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialise subscriber
        self.tf_subscription = self.create_subscription(Odometry, ArculeeTopics.ODOMETRY.value, self.tf_callback, 10)

        self.get_logger().info("Initialised")

    def broadcast_odom_to_world(self):
        static_tf_msg = TransformStamped()

        # Broadcast 0 from odom -> world
        static_tf_msg.header.frame_id = "world"
        static_tf_msg.child_frame_id = "odom"

        static_tf_msg.transform.translation.x = 0.0
        static_tf_msg.transform.translation.y = 0.0
        static_tf_msg.transform.translation.z = 0.0

        static_tf_msg.transform.rotation.x = 0.0
        static_tf_msg.transform.rotation.y = 0.0
        static_tf_msg.transform.rotation.z = 0.0
        static_tf_msg.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(static_tf_msg)

    # Broadcast msg Odometry -> TransformStamped
    def tf_callback(self, msg: Odometry):
        transform_msg = TransformStamped()
        transform_msg.header = msg.header
        transform_msg.child_frame_id = msg.child_frame_id

        transform_msg.transform.translation.x = msg.pose.pose.position.x
        transform_msg.transform.translation.y = msg.pose.pose.position.y
        transform_msg.transform.translation.z = msg.pose.pose.position.z

        transform_msg.transform.rotation.x = msg.pose.pose.orientation.x
        transform_msg.transform.rotation.y = msg.pose.pose.orientation.y
        transform_msg.transform.rotation.z = msg.pose.pose.orientation.z
        transform_msg.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ArculeeWorldBroadcaster()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
