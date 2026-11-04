#!/usr/bin/env python3
"""Smoke-test the running yolo_node and report which inference backend it uses.

Publishes a still frame on the camera topic, calls the ``/yolo/detect`` service,
and prints the node's configured backend plus the detections it returned.  Use it
to prove the ONNX (or pytorch) path works without launching the whole simulation.

Typical use - start the node in one terminal::

    ros2 run rs1_perception yolo_node.py --ros-args -p backend:=onnx -p confidence_threshold:=0.01

then in another::

    ros2 run rs1_perception test_yolo_backend.py --expect-backend onnx

.. note::
   The still frame is a terrain texture, *not* a tree scene, so the detections it
   returns are meaningless noise - this verifies the inference plumbing, not
   detection accuracy.  The low ``confidence_threshold`` above is only so that
   something comes back at all.  For real accuracy use the simulation with
   ``test_tree_locations.py``.
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import Image
from rs1_interfaces.srv import DetectYolo

DEFAULT_IMAGE = (
    "/home/tim/git/robotics-studio-1/rs1_ws/src/rs1_sim/models/forest_plane"
    "/materials/textures/forest_overhead.png"
)

# Parameters worth reporting, in display order.
_INTERESTING_PARAMS = (
    "backend",
    "provider",
    "imgsz",
    "confidence_threshold",
    "iou_threshold",
    "model_path",
    "onnx_model_path",
    "publish_debug_image",
)


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Smoke-test the running yolo_node and report its inference backend.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--image", default=DEFAULT_IMAGE, help="Still frame to publish.")
    p.add_argument(
        "--topic",
        default="/parrot1/camera/image",
        help="Camera image topic the node subscribes to.",
    )
    p.add_argument("--service", default="/yolo/detect", help="Detection service.")
    p.add_argument("--node", default="/yolo_node", help="Name of the yolo node to inspect.")
    p.add_argument(
        "--expect-backend",
        default=None,
        choices=["auto", "pytorch", "onnx", "tensorrt"],
        help="Fail unless the node is configured with this backend.",
    )
    p.add_argument(
        "--min-detections",
        type=int,
        default=1,
        help="Fail if fewer detections come back (0 to accept none).",
    )
    p.add_argument(
        "--frames",
        type=int,
        default=10,
        help="Number of frames to publish before calling the service.",
    )
    return p.parse_args(remove_ros_args(argv)[1:])


def load_frame(path: str) -> np.ndarray:
    """Load ``path`` and resize it to the Parrot camera resolution (720x480)."""
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        print(f"! could not read {path}; using a synthetic frame instead")
        image = np.full((480, 720, 3), 190, np.uint8)
        image[240:, :] = 120
        for i in range(8):
            image[150:430, 40 + i * 85 : 100 + i * 85] = 40
    return cv2.resize(image, (720, 480), interpolation=cv2.INTER_LINEAR)


def _render(value) -> str:
    """Best-effort human rendering of a rcl_interfaces ParameterValue."""
    for attr in ("string_value", "integer_value", "double_value", "bool_value"):
        candidate = getattr(value, attr, None)
        if candidate not in (None, "", 0, 0.0, False):
            return str(candidate)
    if getattr(value, "string_array_value", None):
        return list(value.string_array_value)
    return "<unset>"


class BackendTester(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("yolo_backend_tester")
        self.args = args
        self.publisher = self.create_publisher(Image, args.topic, 10)
        self.client = self.create_client(DetectYolo, args.service)

    # -- parameter introspection ------------------------------------------------
    def read_node_params(self) -> dict:
        """Ask the yolo node for its parameters (needs rcl_interfaces)."""
        try:
            from rcl_interfaces.srv import GetParameters
        except ImportError:
            return {}

        service = f"{self.args.node}/get_parameters"
        client = self.create_client(GetParameters, service)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"! {service} unavailable - cannot read the node's parameters")
            return {}

        request = GetParameters.Request()
        request.names = list(_INTERESTING_PARAMS)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            print(f"! no response from {service}")
            return {}

        return {
            name: _render(value)
            for name, value in zip(_INTERESTING_PARAMS, future.result().values)
        }

    # -- image publishing -------------------------------------------------------
    @staticmethod
    def to_image(bgr: np.ndarray, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = bgr.shape[1] * 3
        msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
        return msg

    def publish_frames(self, bgr: np.ndarray) -> None:
        for _ in range(max(1, self.args.frames)):
            msg = self.to_image(bgr, self.get_clock().now().to_msg())
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def call_detect(self):
        request = DetectYolo.Request()
        request.start_yolo = True
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        return future.result()


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv)

    frame = load_frame(args.image)
    print(f"frame      : {frame.shape} from {args.image}")

    rclpy.init()
    node = BackendTester(args)
    problems: list[str] = []

    try:
        params = node.read_node_params()
        if params:
            print(f"\n{args.node} configuration:")
            for name in _INTERESTING_PARAMS:
                if name not in params:
                    continue
                value = params[name]
                if name == "onnx_model_path" and value == "<unset>":
                    value = "(empty -> derived from model_path)"
                print(f"  {name:<22} {value}")
            configured = str(params.get("backend", ""))
            if args.expect_backend and configured != args.expect_backend:
                problems.append(
                    f"node backend is '{configured}', expected '{args.expect_backend}'"
                )
        else:
            problems.append("could not read the node's parameters")

        if not node.client.wait_for_service(timeout_sec=15.0):
            print(f"\n! {args.service} not available - is yolo_node running?")
            return 1

        print(f"\npublishing {args.frames} frame(s) on {args.topic} ...")
        node.publish_frames(frame)

        print(f"calling {args.service} ...")
        result = node.call_detect()
        if result is None:
            problems.append("the detect service returned nothing")
            boxes = []
        else:
            boxes = result.detections.bounding_boxes

        print(f"\nreturned {len(boxes)} detection(s)")
        for box in boxes[:20]:
            print(
                f"  {box.class_name:<10} {box.score:.4f}  "
                f"[{box.left},{box.top},{box.right},{box.bottom}]"
            )
        if len(boxes) > 20:
            print(f"  ... and {len(boxes) - 20} more")

        # Sanity: boxes must be in the original frame, not letterbox coordinates.
        if boxes:
            xs = [(b.left, b.top, b.right, b.bottom) for b in boxes]
            out_of_range = [
                b for b in xs if b[0] < 0 or b[1] < 0 or b[2] > frame.shape[1] or b[3] > frame.shape[0]
            ]
            if out_of_range:
                problems.append(f"{len(out_of_range)} box(es) fall outside the 720x480 frame")
            else:
                print(f"  all boxes are inside the {frame.shape[1]}x{frame.shape[0]} frame  OK")

        if len(boxes) < args.min_detections:
            problems.append(
                f"got {len(boxes)} detections, expected >= {args.min_detections} "
                "(a still texture is not a tree scene - try "
                "-p confidence_threshold:=0.01, or test with the simulation)"
            )

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    if problems:
        print("RESULT: FAIL")
        for problem in problems:
            print(f"  - {problem}")
        return 1

    print("RESULT: PASS - the service answered and returned in-frame detections")
    return 0


if __name__ == "__main__":
    sys.exit(main())
