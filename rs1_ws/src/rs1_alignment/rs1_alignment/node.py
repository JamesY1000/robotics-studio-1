"""One-way /robot/tf -> /tf; truth is never inserted into the live TF tree."""
import json
import math
from collections import deque
from uuid import uuid4

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException

from .geometry import compose, registration, wrap, yaw

ROBOTS = ("husky1", "parrot1")


class Alignment(Node):
    def __init__(self):
        super().__init__("world_alignment")
        self.session = self.declare_parameter("session_id", "manual").value
        self.instance = str(uuid4())
        self.world = self.declare_parameter("world_name", "v2_forest").value
        self.xy_tolerance = self.declare_parameter("xy_tolerance", 0.25).value
        self.yaw_tolerance = self.declare_parameter("yaw_tolerance", 0.0872665).value
        self.buffers = {r: Buffer(cache_time=Duration(seconds=20)) for r in ROBOTS}
        self.pending = {r: deque(maxlen=200) for r in ROBOTS}
        self.transforms = {}
        self.checks = {r: 0 for r in ROBOTS}
        self.errors = {}
        self.last_verified = {r: 0 for r in ROBOTS}
        self.initial_verification = {}
        self.offsets = {r: self.declare_parameter(f"{r}_model_to_base", [0.0, 0.0, 0.0]).value for r in ROBOTS}
        self.dynamic_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.static_pub = StaticTransformBroadcaster(self)
        self.statics = {}
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/mission/alignment", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscriptions_owned = []
        for robot in ROBOTS:
            self.subscriptions_owned += [
                self.create_subscription(TFMessage, f"/{robot}/tf", lambda m, r=robot: self.receive_tf(r, m, False), 100),
                self.create_subscription(TFMessage, f"/{robot}/tf_static", lambda m, r=robot: self.receive_tf(r, m, True), latched),
                self.create_subscription(TFMessage, f"/mission/truth/{robot}", lambda m, r=robot: self.truth(r, m), 20),
            ]
        self.last_clock = 0
        self.invalid = False
        self.create_timer(0.1, self.tick)

    def receive_tf(self, robot, message, static):
        accepted = []
        for transform in message.transforms:
            # Only this robot's local tree, no world registrations or foreign parents.
            if not (transform.header.frame_id.startswith(robot+"_") and transform.child_frame_id.startswith(robot+"_")):
                continue
            accepted.append(transform)
            if static:
                self.buffers[robot].set_transform_static(transform, "robot_description")
                self.statics[transform.child_frame_id] = transform
            else:
                self.buffers[robot].set_transform(transform, "localization")
        if static:
            self.static_pub.sendTransform(list(self.statics.values()) + list(self.transforms.values()))
        elif accepted:
            self.dynamic_pub.publish(TFMessage(transforms=accepted))

    def truth(self, robot, message):
        for t in message.transforms:
            # PosePublisher emits the world-to-model pose with scoped names.
            if t.child_frame_id.split("::")[-1] == robot and t.header.frame_id in (self.world, "world"):
                self.pending[robot].append(t)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            self.invalid = True
            self.errors["clock"] = "Simulation clock reset. Use New session; static alignment cannot be reused."
        self.last_clock = now
        if not self.invalid:
            for robot in ROBOTS:
                queue = self.pending[robot]
                # Retry recent truth until matching historical localization TF exists.
                while queue:
                    truth = queue[0]
                    stamp = Time.from_msg(truth.header.stamp)
                    if now-stamp.nanoseconds > 5_000_000_000:
                        queue.popleft()
                        continue
                    try:
                        local = self.buffers[robot].lookup_transform(robot+"_map", robot+"_base_link", stamp)
                    except TransformException:
                        # A newer timestamp may fit the buffer if this predates startup.
                        if len(queue) > 30:
                            queue.popleft()
                            continue
                        break
                    queue.popleft()
                    wp, wq = truth.transform.translation, truth.transform.rotation
                    lp, lq = local.transform.translation, local.transform.rotation
                    world_base = compose((wp.x, wp.y, yaw(wq)), self.offsets[robot])
                    map_base = (lp.x, lp.y, yaw(lq))
                    if not all(math.isfinite(v) for v in (*world_base, *map_base)):
                        continue
                    if robot not in self.transforms:
                        x, y, theta = registration(world_base, map_base)
                        t = TransformStamped()
                        t.header.stamp = truth.header.stamp
                        t.header.frame_id = "world"
                        t.child_frame_id = robot+"_map"
                        t.transform.translation.x, t.transform.translation.y = x, y
                        # Planar registration only: map and world Z datums coincide.
                        t.transform.translation.z = 0.0
                        t.transform.rotation.z, t.transform.rotation.w = math.sin(theta/2), math.cos(theta/2)
                        self.transforms[robot] = t
                        self.static_pub.sendTransform(list(self.statics.values()) + list(self.transforms.values()))
                    t = self.transforms[robot].transform
                    estimated = compose((t.translation.x, t.translation.y, yaw(t.rotation)), map_base)
                    error_xy = math.hypot(estimated[0]-world_base[0], estimated[1]-world_base[1])
                    error_yaw = abs(wrap(estimated[2]-world_base[2]))
                    good = error_xy <= self.xy_tolerance and error_yaw <= self.yaw_tolerance
                    # Verify independent subsequent samples, without continually realigning.
                    self.checks[robot] = self.checks[robot]+1 if good else 0
                    self.last_verified[robot] = stamp.nanoseconds
                    self.errors[robot] = {"xy_m": error_xy, "yaw_rad": error_yaw}
                    if self.checks[robot] >= 10 and robot not in self.initial_verification:
                        self.initial_verification[robot] = {
                            "stamp": stamp.nanoseconds / 1e9,
                            "xy_m": error_xy, "yaw_rad": error_yaw,
                            "world_base": world_base, "map_base": map_base,
                        }
        ready = {r: not self.invalid and self.checks[r] >= 10 and
                 now-self.last_verified[r] < 2_000_000_000 for r in ROBOTS}
        self.status_pub.publish(String(data=json.dumps({
            "session_id": self.session, "instance_id": self.instance, "world_name": self.world,
            "ready": ready, "errors": self.errors,
            "registered": list(self.transforms), "invalid": self.invalid,
            "initial_verification": self.initial_verification,
        })))


def main():
    rclpy.init()
    node = Alignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
