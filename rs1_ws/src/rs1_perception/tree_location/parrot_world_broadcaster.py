"""Broadcasts the Parrot drone's TF tree onto the global ``/tf`` so the
tree-location node can look up ``world -> parrot1_camera_*_optical_frame``.

The ``rs1_sim`` launch runs ``robot_state_publisher`` and ``robot_localization``
inside the ``/parrot1`` namespace, so their transforms are published on
``/parrot1/tf`` and ``/parrot1/tf_static`` (not the global ``/tf``). There is
also no global ``world`` frame at all. This node:

* publishes ``world -> parrot1_base_link`` from the Gazebo odometry (the drone
  can be teleported around the forest), with the spawn pose as an initial
  fallback, and
* republishes ``/parrot1/tf_static`` onto the global ``/tf_static`` (the fixed
  ``base_link -> camera_link -> camera_*_optical_frame`` chain),

so the full chain ``world -> parrot1_base_link -> ... -> camera`` exists and the
camera pose matches where the drone actually sits in Gazebo.

The EKF's ``/parrot1/tf`` (``parrot1_odom -> parrot1_base_link``) is
DELIBERATELY not republished - it only integrates sensor noise and drifts. The
raw Gazebo odometry is exact instead (``ignition::gazebo::systems::OdometryPublisher``
publishes the world pose of the model).

Self-contained: only uses rclpy / tf2_ros.
"""

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _yaw_to_quaternion(yaw: float) -> tuple:
    import math

    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ParrotWorldBroadcaster(Node):
    def __init__(self):
        super().__init__("parrot_world_broadcaster")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_link_frame", "parrot1_base_link")
        self.declare_parameter("tf_static_topic", "/parrot1/tf_static")
        self.declare_parameter("odometry_topic", "/parrot1/odometry")
        # Drone spawn pose [x, y, z, yaw] - used only until the first odometry
        # message arrives (rs1_sim spawns parrot1 at [90, -48, 8, 0]).
        self.declare_parameter("spawn_pose", [90.0, -48.0, 8.0, 0.0])

        self.world_frame = self.get_parameter("world_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.tf_static_topic = self.get_parameter("tf_static_topic").value
        self.odometry_topic = self.get_parameter("odometry_topic").value
        spawn = list(self.get_parameter("spawn_pose").value)

        # world -> base_link is published dynamically from Gazebo odometry.
        self.tf_broadcaster = TransformBroadcaster(self)
        self._publish_world_to_base(spawn[0], spawn[1], spawn[2], _yaw_to_quaternion(spawn[3]))

        # Republish the namespaced static TF chain (base_link -> ... -> camera)
        # onto the global /tf_static.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.parrot_statics: List[TransformStamped] = []
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            self.tf_static_topic,
            self.tf_static_cb,
            QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        self.odom_sub = self.create_subscription(
            Odometry, self.odometry_topic, self.odom_cb, 10
        )

        self.get_logger().info(
            f"Broadcasting {self.world_frame} -> {self.base_link_frame} from "
            f"{self.odometry_topic}; republishing {self.tf_static_topic} to /tf_static"
        )

    def _publish_world_to_base(self, x, y, z, q) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._publish_world_to_base(p.x, p.y, p.z, (q.x, q.y, q.z, q.w))

    def tf_static_cb(self, msg: TFMessage) -> None:
        # StaticTransformBroadcaster keeps only the last message, so republish
        # the whole latched chain each time it changes.
        self.parrot_statics = list(msg.transforms)
        if self.parrot_statics:
            self.static_broadcaster.sendTransform(self.parrot_statics)


def main(args=None):
    rclpy.init(args=args)
    node = ParrotWorldBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
