#!/usr/bin/env python3
"""End-to-end test for the YOLO -> TreeLocation pipeline in the rs1_sim forest.

For each of several viewpoints the script:

1. teleports the Parrot to a spot next to a known tree (unless --no-teleport),
2. calls the ``/yolo/detect`` service to get ``YoloDetections``,
3. feeds those detections into ``/yolo/process_tree_locations`` to get
   ``TreeLocation[]``,
4. compares each reported tree base against the ground-truth tree poses parsed
   from the ``v2_forest.sdf`` world file.

The YOLO model is only ~80% accurate, so missed trees are expected and are NOT
counted as failures. What matters is precision: every returned location must
match a ground-truth tree of the same type within ``--error-threshold`` metres.

Run inside the sourced environment, with the sim and perception launched:

    ros2 run rs1_perception test_tree_locations.py

Exit code is 0 when precision (returned locations matching a ground-truth tree)
is at least ``--min-precision`` and at least one tree was returned.
"""

from __future__ import annotations

import argparse
import math
import random
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener

from rs1_interfaces.msg import YoloDetections
from rs1_interfaces.srv import DetectYolo, ProcessTreeLocations


@dataclass
class GroundTruthTree:
    name: str
    tree_type: str  # OAK_TREE | PINE_TREE
    x: float
    y: float
    z: float


def parse_world_trees(world_file: Path) -> List[GroundTruthTree]:
    text = world_file.read_text()
    trees: List[GroundTruthTree] = []
    for block in re.findall(r"<include>(.*?)</include>", text, re.S):
        uri = re.search(r"<uri>\s*([^<]+?)\s*</uri>", block)
        name = re.search(r"<name>\s*([^<]+?)\s*</name>", block)
        pose = re.search(r"<pose>\s*([^<]+?)\s*</pose>", block)
        if not (uri and name and pose):
            continue
        uri_s = uri.group(1)
        if "trees/oak_tree" in uri_s:
            ttype = "OAK_TREE"
        elif "trees/pine_tree" in uri_s:
            ttype = "PINE_TREE"
        else:
            continue
        vals = [float(v) for v in pose.group(1).split()]
        trees.append(GroundTruthTree(name.group(1), ttype, vals[0], vals[1], vals[2]))
    return trees


def set_pose(model_name: str, world_name: str, x: float, y: float, z: float, yaw: float) -> bool:
    hz = yaw * 0.5
    req = (
        f'name: "{model_name}", '
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {math.sin(hz)}, w: {math.cos(hz)}}}"
    )
    cmd = [
        "ign", "service", "-s", f"/world/{world_name}/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "5000", "--req", req,
    ]
    for _ in range(3):
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode == 0 and "true" in (res.stdout + res.stderr).lower():
            return True
        time.sleep(0.5)
    return False


def choose_viewpoints(trees: List[GroundTruthTree], count: int, min_sep: float, view_distance: float) -> List[GroundTruthTree]:
    ordered = list(trees)
    random.shuffle(ordered)
    chosen: List[GroundTruthTree] = []
    for t in ordered:
        if all(math.hypot(t.x - c.x, t.y - c.y) >= min_sep for c in chosen):
            chosen.append(t)
        if len(chosen) >= count:
            break
    return chosen


class TreeLocationTester(Node):
    def __init__(self, args) -> None:
        super().__init__("tree_location_tester")
        self.args = args

        self.detect_client = self.create_client(DetectYolo, "/yolo/detect")
        self.process_client = self.create_client(ProcessTreeLocations, "/yolo/process_tree_locations")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _call(self, client, request, timeout: float = 60.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service {client.srv_name} unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"Service call to {client.srv_name} failed")
        return future.result()

    def wait_for_base_pose(self, target_xy, tolerance: float = 1.0, timeout: float = 15.0) -> bool:
        """Wait until the world->base_link TF reports the teleported position."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform("world", "parrot1_base_link", Time(), timeout=Duration(seconds=0.5))
                dx = tf.transform.translation.x - target_xy[0]
                dy = tf.transform.translation.y - target_xy[1]
                if math.hypot(dx, dy) <= tolerance:
                    return True
            except Exception:  # noqa: BLE001
                pass
        return False

    def detect(self) -> YoloDetections:
        req = DetectYolo.Request()
        req.start_yolo = True
        return self._call(self.detect_client, req).detections

    def process(self, detections: YoloDetections):
        req = ProcessTreeLocations.Request()
        req.start_lidar = True
        req.detections = detections
        return self._call(self.process_client, req).tree_data


def main() -> int:
    ap = argparse.ArgumentParser(description="Test the YOLO -> TreeLocation pipeline.")
    ap.add_argument("--world", default="v2_forest")
    ap.add_argument("--world-file", default="")
    ap.add_argument("--model-name", default="parrot1")
    ap.add_argument("--views", type=int, default=4)
    ap.add_argument("--view-distance", type=float, default=15.0)
    ap.add_argument("--camera-height", type=float, default=4.0)
    ap.add_argument("--min-separation", type=float, default=40.0)
    ap.add_argument("--settle-time", type=float, default=2.5)
    ap.add_argument("--error-threshold", type=float, default=4.0,
                    help="Horizontal error accepted as a match to a ground-truth tree (m).")
    ap.add_argument("--min-precision", type=float, default=0.9,
                    help="Minimum fraction of returned locations that must match a tree.")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    random.seed(args.seed)

    if args.world_file:
        world_file = Path(args.world_file)
    else:
        world_file = Path(get_package_share_directory("rs1_sim")) / "worlds" / f"{args.world}.sdf"
    if not world_file.is_file():
        print(f"World file not found: {world_file}")
        return 2
    trees = parse_world_trees(world_file)
    print(f"Parsed {len(trees)} ground-truth trees from {world_file}")
    if not trees:
        return 2

    viewpoints = choose_viewpoints(trees, args.views, args.min_separation, args.view_distance)
    if args.no_teleport:
        viewpoints = [None]  # test at the current pose only

    rclpy.init()
    node = TreeLocationTester(args)
    failures = 0
    total_returned = 0
    total_correct = 0
    total_type_correct = 0
    errors: List[float] = []

    try:
        for idx, target in enumerate(viewpoints):
            if target is not None:
                # Stand off the tree, keeping the drone inside the walls.
                if target.x >= 0:
                    drone_x, drone_y, yaw = target.x - args.view_distance, target.y, 0.0
                else:
                    drone_x, drone_y, yaw = target.x + args.view_distance, target.y, math.pi
                drone_z = target.z + args.camera_height
                drone_x = max(-95.0, min(95.0, drone_x))
                drone_y = max(-95.0, min(95.0, drone_y))
                print(f"\n[{idx + 1}/{len(viewpoints)}] Teleport to ({drone_x:.1f}, {drone_y:.1f}, {drone_z:.1f}) "
                      f"target={target.name} ({target.tree_type}) @ ({target.x:.1f}, {target.y:.1f})")
                if not set_pose(args.model_name, args.world, drone_x, drone_y, drone_z, yaw):
                    print("  set_pose failed; skipping viewpoint")
                    failures += 1
                    continue
                if not node.wait_for_base_pose((drone_x, drone_y)):
                    print("  warning: TF did not report the teleported pose in time")
                time.sleep(args.settle_time)
            else:
                print(f"\n[{idx + 1}/{len(viewpoints)}] Using current drone pose")

            detections = node.detect()
            print(f"  YOLO detections: {len(detections.bounding_boxes)}")
            if not detections.bounding_boxes:
                continue

            locations = node.process(detections)
            print(f"  Tree locations returned: {len(locations)}")

            for loc in locations:
                total_returned += 1
                # Nearest ground-truth tree of ANY type (location accuracy), and
                # of the SAME type (also requires the label to be right).
                best_any = min(
                    trees, key=lambda t: math.hypot(t.x - loc.location.x, t.y - loc.location.y)
                )
                err_any = math.hypot(best_any.x - loc.location.x, best_any.y - loc.location.y)
                same_type = [t for t in trees if t.tree_type == loc.tree_type]
                if same_type:
                    best_same = min(
                        same_type, key=lambda t: math.hypot(t.x - loc.location.x, t.y - loc.location.y)
                    )
                    err_same = math.hypot(best_same.x - loc.location.x, best_same.y - loc.location.y)
                else:
                    err_same = float("inf")

                errors.append(err_any)
                ok_loc = err_any <= args.error_threshold
                ok_type = err_same <= args.error_threshold
                total_correct += 1 if ok_loc else 0
                total_type_correct += 1 if ok_type else 0
                if not ok_loc:
                    failures += 1
                type_note = (
                    "" if best_any.tree_type == loc.tree_type
                    else f" (label {loc.tree_type}, actually {best_any.tree_type})"
                )
                print(f"    - {loc.tree_type} at ({loc.location.x:.1f}, {loc.location.y:.1f}, "
                      f"{loc.location.z:.1f}) -> nearest GT {best_any.name} err={err_any:.2f} m "
                      f"{'OK' if ok_loc else 'FAIL'}{type_note}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n==== Summary ====")
    print(f"Locations returned: {total_returned}")
    print(f"Location precision (nearest tree of ANY type <= {args.error_threshold} m): "
          f"{total_correct}/{total_returned}")
    precision = total_correct / total_returned if total_returned else 0.0
    print(f"Precision: {precision:.2f}")
    if total_returned:
        print(f"Type-correct precision (label matches the tree's type): "
              f"{total_type_correct}/{total_returned} "
              f"({total_type_correct / total_returned:.2f})")
    if errors:
        print(f"Mean horizontal error: {sum(errors) / len(errors):.2f} m   Max: {max(errors):.2f} m")
    print("Note: recall is limited by the YOLO model (~80%); missed/mislabelled trees are expected.")
    if total_returned == 0:
        print("FAIL: no tree locations were returned")
        return 1
    if precision < args.min_precision:
        print(f"FAIL: precision {precision:.2f} < required {args.min_precision:.2f} ({failures} problem(s))")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
