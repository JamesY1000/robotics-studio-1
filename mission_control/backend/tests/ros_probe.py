"""Runtime smoke probe. Run in a sourced ROS environment against a live session.

python mission_control/backend/tests/ros_probe.py
Checks retained registration, single parents, matched-time truth errors and
world ROI -> executor map -> world conversion. Does not command either robot.
"""
import json
import math
import time
from collections import deque

import httpx
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # registers PoseStamped conversions


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    state = httpx.get("http://127.0.0.1:8000/api/state").json()
    assert state["plan"], "Start a dashboard session first"
    rclpy.init()
    node = Node("mission_control_smoke_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    pending = {r: deque(maxlen=100) for r in ("husky1", "parrot1")}
    errors = {r: [] for r in pending}
    parents = {}
    conflicts = []
    def inspect(message):
        for t in message.transforms:
            if t.child_frame_id in parents and parents[t.child_frame_id] != t.header.frame_id:
                conflicts.append(t.child_frame_id)
            parents[t.child_frame_id] = t.header.frame_id
    node.create_subscription(TFMessage, "/tf", inspect, 100)
    from rclpy.qos import QoSProfile, DurabilityPolicy
    node.create_subscription(TFMessage, "/tf_static", inspect, QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for robot in pending:
        node.create_subscription(TFMessage, f"/mission/truth/{robot}", lambda m, r=robot: pending[r].extend(m.transforms), 20)
    deadline = time.monotonic()+10
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            for robot, queue in pending.items():
                while queue:
                    truth = queue[0]
                    if truth.child_frame_id != robot:
                        queue.popleft(); continue
                    try:
                        estimate = buffer.lookup_transform("world", robot+"_base_link", Time.from_msg(truth.header.stamp))
                    except TransformException:
                        if len(queue) > 30: queue.popleft(); continue
                        break
                    queue.popleft()
                    a, b = truth.transform, estimate.transform
                    errors[robot].append([math.hypot(a.translation.x-b.translation.x, a.translation.y-b.translation.y),
                                         abs(math.atan2(math.sin(yaw(a.rotation)-yaw(b.rotation)), math.cos(yaw(a.rotation)-yaw(b.rotation))))])
        assert not conflicts, conflicts
        for r in pending:
            assert parents[r+"_base_link"] == r+"_odom", parents
            assert parents[r+"_odom"] == r+"_map", parents
            assert parents[r+"_map"] == "world", parents
            assert len(errors[r]) >= 10, errors
        roi = PoseStamped()
        roi.header.frame_id = "world"
        roi.pose.position.x = state["plan"]["roi"]["x"]
        roi.pose.position.y = state["plan"]["roi"]["y"]
        roi.pose.orientation.w = 1.0
        local = buffer.transform(roi, "parrot1_map")
        back = buffer.transform(local, "world")
        assert math.hypot(back.pose.position.x-roi.pose.position.x, back.pose.position.y-roi.pose.position.y) < 1e-6
        print(json.dumps({"session_id": state["plan"]["session_id"], "phase": state["phase"],
                          "samples": {r: len(e) for r,e in errors.items()},
                          "max_xy_m_yaw_rad": {r: [max(e[0] for e in es), max(e[1] for e in es)] for r,es in errors.items()},
                          "roi_world": [roi.pose.position.x, roi.pose.position.y],
                          "roi_parrot_map": [local.pose.position.x, local.pose.position.y],
                          "single_parent_frames": len(parents), "roi_round_trip": "passed"}, indent=2))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
