#!/usr/bin/env python3
"""Broadcasts the Parrot drone's TF tree onto the global ``/tf`` so the sample
generator can look up ``world -> camera``.

The rs1_ws ``41068_ignition_bringup`` runs ``robot_state_publisher`` and
``robot_localization`` inside the ``/parrot1`` namespace, so their transforms are
published on ``/parrot1/tf`` and ``/parrot1/tf_static`` (not the global ``/tf``).
This node:

* publishes a static ``world -> parrot1_base_link`` transform equal to the
  drone's spawn pose, and
* republishes ``/parrot1/tf_static`` -> ``/tf_static`` (the fixed ``base_link ->
  ... -> camera_rgb_optical_frame`` chain), so the full chain ``world ->
  parrot1_base_link -> parrot1_camera_rgb_optical_frame`` exists and the camera
  pose matches where the drone actually sits in Gazebo.

Note: the EKF's ``/parrot1/tf`` (``parrot1_odom -> parrot1_base_link``) is
DELIBERATELY not republished. The drone is static (physics is disabled), so the
EKF only integrates sensor noise and drifts several metres away from the true
spawn pose, which offset every projected bounding box. The static
``world -> parrot1_base_link`` is exact instead.

Self-contained: only uses rclpy.
"""

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

NODE_NAME = "parrot_world_broadcaster_node"

# robot_state_publisher publishes /tf_static with TRANSIENT_LOCAL (latched)
# durability; a default (volatile) subscription would miss the latched message.
_TF_STATIC_QOS = QoSProfile(
    depth=100,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class ParrotWorldBroadcaster(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_link_frame", "parrot1_base_link")
        self.declare_parameter("tf_static_topic", "/parrot1/tf_static")
        # The drone's spawn pose [x, y, z, yaw] in the Gazebo world - must match
        # the pose used to spawn the parrot (41068_ignition.launch.py default).
        self.declare_parameter("spawn_pose", [2.0, 0.0, 0.8, 0.0])

        self.world_frame = self.get_parameter("world_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.tf_static_topic = self.get_parameter("tf_static_topic").value
        spawn = self.get_parameter("spawn_pose").value

        # world -> base_link (drone spawn pose)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        static_msg = self._make_static_transform(spawn)
        # Static transforms should be stamped with time 0 so tf2 treats them as
        # always-valid regardless of the sim clock.
        static_msg.header.stamp = rclpy.time.Time().to_msg()
        self.world_to_base = static_msg
        # The StaticTransformBroadcaster is a LATCHED publisher that keeps only
        # the last message it was given, so we must re-send the world->base_link
        # link together with the republished parrot static chain (otherwise it
        # gets overwritten and the TF tree disconnects at parrot1_base_link).
        self.parrot_statics: List[TransformStamped] = []
        self._publish_all_static()
        self.get_logger().info(f"Broadcast static {self.world_frame} -> {self.base_link_frame} at {spawn}")

        # Republish the namespaced static TF (base_link -> ... -> camera) onto
        # the global /tf_static. The EKF's /parrot1/tf is intentionally ignored
        # (see module docstring).
        self.tf_static_sub = self.create_subscription(TFMessage, self.tf_static_topic, self.tf_static_cb, _TF_STATIC_QOS)
        self.get_logger().info("Initialized")

    def _publish_all_static(self) -> None:
        self.static_broadcaster.sendTransform([self.world_to_base] + self.parrot_statics)

    def _make_static_transform(self, spawn: List[float]) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_link_frame
        msg.transform.translation.x = float(spawn[0])
        msg.transform.translation.y = float(spawn[1])
        msg.transform.translation.z = float(spawn[2])
        q = quaternion_from_euler(0.0, 0.0, float(spawn[3]))
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        return msg

    def tf_static_cb(self, msg: TFMessage) -> None:
        self.parrot_statics = list(msg.transforms)
        self._publish_all_static()


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
