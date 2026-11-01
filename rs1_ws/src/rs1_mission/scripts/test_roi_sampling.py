#!/usr/bin/env python3
"""Parrot-only ROI sampling test for the lawnmower (SampleROI) mission phase.

Spawns nothing itself: with only the parrot in the world, the perception stack
and the parrot mission executor running, it sends a single ``SampleROI`` goal to
``/parrot1/sample_roi`` for a preset ROI box and scores the resulting tree map
against the ground-truth trees inside that box.

The parrot (via Nav2 ``NavigateThroughPoses``) flies the boustrophedon sweep at
the requested search clearance, continuously running the perception services.
The merged result arrives on the latched ``/parrot1/tree_map`` topic.

The Nav2 costmap is rarely complete over a fresh ROI, so the executor first
drives the phase-1 weighted frontier explorer (``/parrot1/explore_to_pose``,
provided by ``explore_lite`` during parrot navigation bringup) toward the ROI to
grow the map, and falls back to it again whenever a sweep leg is rejected or
aborted. This means the test needs the full parrot navigation stack running.

By default the drone is NOT teleported: it flies from wherever it is to the ROI
corner nearest to it, and the boustrophedon starts from that corner (the executor
picks the corner and the lane order/direction).  Pass ``--teleport`` to place the
drone at the nearest corner instantly instead, which skips the transit but needs
a SLAM re-initialisation after the jump.

Metrics (gated):
  * location precision: fraction of returned trees within ``--error-threshold``
    of a ground-truth tree of ANY type;
  * type precision: same, but the nearest ground truth must also share the type;
  * recall: fraction of ground-truth trees in the ROI matched by a returned tree.

Run inside the sourced workspace (parrot-only simulation):

    # Terminal 1 - parrot only, with navigation for the sweep
    ros2 launch rs1_sim rs1_sim.launch.py husky:=false rviz:=false headless:=True

    # Terminal 2 - perception
    ros2 launch rs1_perception tree_identification.launch.py

    # Terminal 3 - parrot mission executor only (no coordinator, no husky)
    ros2 launch rs1_mission roi_sampling_test.launch.py

    # Terminal 4 - the test
    ros2 run rs1_mission test_roi_sampling.py --roi 62 -88 92 -58

Exit code is 0 when all gates pass, 1 when a gate fails, 2 on setup errors.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RosDuration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rclpy.utilities import remove_ros_args

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from rs1_interfaces.action import SampleROI
from rs1_interfaces.msg import TreeMap

# A dense mixed oak/pine cluster next to the parrot spawn (90, -48).
DEFAULT_ROI = (62.0, -88.0, 92.0, -58.0)


@dataclass
class GroundTruthTree:
    name: str
    tree_type: str  # OAK_TREE | PINE_TREE
    x: float
    y: float
    z: float


def set_pose(model_name: str, world_name: str, x: float, y: float, z: float, yaw: float) -> bool:
    """Teleport an existing Gazebo model to a pose in the world."""
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


def parse_world_trees(world_file: Path) -> List[GroundTruthTree]:
    """Ground-truth trees parsed from the world SDF (same rules as perception tests)."""
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


def trees_in_box(
    trees: List[GroundTruthTree],
    bounds: Tuple[float, float, float, float],
    margin: float,
) -> List[GroundTruthTree]:
    x_min, y_min, x_max, y_max = bounds
    return [
        t
        for t in trees
        if x_min - margin <= t.x <= x_max + margin
        and y_min - margin <= t.y <= y_max + margin
    ]


class RoiSamplingTester(Node):
    def __init__(self, world_name: str = "v2_forest") -> None:
        super().__init__("roi_sampling_tester")
        self.world_name = world_name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/parrot1/initialpose",
            10,
        )
        self.client: ActionClient = ActionClient(self, SampleROI, "/parrot1/sample_roi")
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.tree_map: Optional[TreeMap] = None
        self.create_subscription(TreeMap, "/parrot1/tree_map", self._on_tree_map, latched)
        # world<->parrot1_map registration status (published by rs1_alignment).
        self.alignment_ready = False
        self.alignment_status = None
        self.create_subscription(String, "/mission/alignment", self._on_alignment, latched)

    def _on_tree_map(self, msg: TreeMap) -> None:
        self.tree_map = msg

    def _on_alignment(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        self.alignment_status = data
        ready = data.get("ready", {})
        self.alignment_ready = bool(ready.get("parrot1", False)) and not data.get("invalid", False)

    def wait_for_alignment(self, timeout: float = 20.0) -> bool:
        """Spin until rs1_alignment reports world<->parrot1_map ready."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.alignment_ready:
                return True
        return False

    def _world_to_map_pose(self, x: float, y: float, z: float, yaw: float):
        """Transform a world-frame pose into parrot1_map, or None if TF is missing."""
        deadline = time.time() + 5.0
        tf = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    "parrot1_map", "world", Time(), RosDuration(seconds=0.5))
                break
            except Exception:  # noqa: BLE001
                continue
        if tf is None:
            return None
        import tf2_geometry_msgs  # noqa: F401
        from tf2_geometry_msgs import do_transform_pose

        src = Pose()
        src.position.x = x
        src.position.y = y
        src.position.z = z
        src.orientation.z = math.sin(yaw * 0.5)
        src.orientation.w = math.cos(yaw * 0.5)
        return do_transform_pose(src, tf)

    def _wait_for_map_pose(self, target, tol: float = 1.5, timeout: float = 5.0) -> bool:
        """Spin until the drone's parrot1_map pose reaches ``target`` (xy)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    "parrot1_map", "parrot1_base_link", Time(), RosDuration(seconds=0.5))
                dx = tf.transform.translation.x - target.x
                dy = tf.transform.translation.y - target.y
                if math.hypot(dx, dy) <= tol:
                    return True
            except Exception:  # noqa: BLE001
                pass
        return False

    def transform_box(
        self, bounds: Tuple[float, float, float, float], source_frame: str
    ) -> Tuple[str, Point, Point]:
        """Return the ROI box in the executor's planning frame (parrot1_odom).

        The sweep no longer plans in parrot1_map: SLAM's map->odom diverges on
        this repetitive forest and drove the drone physically out of the ROI.
        parrot1_odom is the EKF output, which fuses the exact Gazebo pose, so it
        is numerically the same as ``world`` and the raw box needs no transform.
        """
        x_min, y_min, x_max, y_max = bounds
        return (
            "parrot1_odom",
            Point(x=x_min, y=y_min, z=0.0),
            Point(x=x_max, y=y_max, z=0.0),
        )

    def _lookup(self, target_frame: str, source_frame: str, timeout: float = 0.5):
        """Look up ``target_frame <- source_frame``, spinning the buffer while waiting."""
        # 10 s, not 3 s: right after a launch the EKF has not published
        # parrot1_odom -> parrot1_base_link yet, so the short window made a
        # perfectly healthy stack report "no parrot1_odom -> parrot1_base_link".
        deadline = time.time() + 10.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time(), RosDuration(seconds=timeout))
            except Exception:  # noqa: BLE001
                continue
        return None

    def _sweep_start(self, box_min, box_max, swath_spacing: float):
        """First swath waypoint (x, y, yaw) inside a box; mirrors computeSwaths()."""
        x_min, y_min, x_max, y_max = box_min[0], box_min[1], box_max[0], box_max[1]
        if not (x_max > x_min and y_max > y_min):
            return None
        # Match the executor: lanes are spaced by the detection range with
        # overlap (NOT the ~55 m forward-footprint width, which left large ROIs
        # almost unswept).
        spacing = swath_spacing if swath_spacing > 0.0 else 20.0 * (1.0 - 0.2)
        spacing = max(spacing, 2.0)
        turn_lead = 3.0  # turn_lead_m_ default in the executor
        if (x_max - x_min) >= (y_max - y_min):
            height = y_max - y_min
            step = height / float(max(1, int(math.ceil(height / spacing))))
            return x_min - turn_lead, y_min + 0.5 * step, 0.0
        width = x_max - x_min
        step = width / float(max(1, int(math.ceil(width / spacing))))
        return x_min + 0.5 * step, y_min - turn_lead, math.pi * 0.5

    def _measured_map_pose(self, odom_transform):
        """Express a measured pose in parrot1_map without trusting SLAM's own correction.

        The EKF fuses the Gazebo ground-truth pose, so ``parrot1_odom`` is
        numerically the ``world`` frame.  Relabelling the measured odom pose as
        ``world`` and applying the *static* ``parrot1_map <- world`` registration
        avoids depending on the (currently suspect) ``map -> odom`` correction.
        """
        tf = self._lookup("parrot1_map", "world")
        if tf is None:
            return None
        import tf2_geometry_msgs  # noqa: F401
        from tf2_geometry_msgs import do_transform_pose

        src = Pose()
        src.position.x = odom_transform.translation.x
        src.position.y = odom_transform.translation.y
        src.position.z = odom_transform.translation.z
        src.orientation = odom_transform.rotation
        return do_transform_pose(src, tf)

    def _publish_initial_pose(self, map_pose) -> None:
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "parrot1_map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose = map_pose
        # Small covariance so slam_toolbox trusts the re-init.
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.07
        self.initial_pose_pub.publish(pose)

    def check_map_frame(self) -> bool:
        """Verify the executor's planning frame (parrot1_odom) is live and sane.

        The sweep now plans in parrot1_odom (the EKF's exact ground-truth pose),
        not parrot1_map, because SLAM's map->odom diverges on this forest and
        drove the drone physically out of the ROI.  This just confirms the
        odometry frame chain exists.
        """
        odom = self._lookup("parrot1_odom", "parrot1_base_link")
        if odom is None:
            print("FAIL: no parrot1_odom -> parrot1_base_link transform; is the EKF running?")
            return False
        self.get_logger().info(
            "Sweep frame parrot1_odom ready (ground-truth pose at "
            f"({odom.transform.translation.x:.1f}, {odom.transform.translation.y:.1f}))")
        return True

    def teleport_to_sweep_start(self, args, roi_min: Point, roi_max: Point) -> bool:
        """Teleport to the first swath waypoint and re-sync localisation, in order."""
        start = self._sweep_start(
            (args.roi[0], args.roi[1]), (args.roi[2], args.roi[3]),
            float(args.swath_spacing))
        if start is None:
            print("WARN: invalid ROI box for sweep-start teleport")
            return True
        start_x, start_y, start_yaw = start

        landing_z = 8.0
        if not set_pose("parrot1", self.world_name, start_x, start_y, landing_z, start_yaw):
            print(f"WARN: failed to teleport to the sweep start ({start_x:.1f}, {start_y:.1f})")
            return True

        # 1. WAIT for the EKF to reflect the teleport BEFORE touching SLAM.
        #    slam_toolbox implements /initialpose as map_T_odom = requested - odom.
        #    Re-initialising while odom still held the pre-teleport pose baked a
        #    ~90 m translation plus a large yaw into map->odom, so the sweep flew
        #    the mirrored box: straight out of the ROI, in the opposite direction.
        measured = None
        settled = 0
        deadline = time.time() + 15.0
        while time.time() < deadline:
            tf = self._lookup("parrot1_odom", "parrot1_base_link")
            if tf is not None:
                dx = tf.transform.translation.x - start_x
                dy = tf.transform.translation.y - start_y
                if math.hypot(dx, dy) <= 1.5:
                    settled += 1
                    measured = tf.transform
                    if settled >= 2:
                        break
                else:
                    settled = 0
            time.sleep(0.1)
        if settled < 2:
            print(
                "FAIL: odometry never settled at the teleport target "
                f"({start_x:.1f}, {start_y:.1f}); is the EKF running?")
            return False

        # 2. Re-initialise SLAM from the MEASURED pose, not the commanded one, so
        #    a residual teleport offset cannot be mistaken for a map correction.
        map_pose = self._measured_map_pose(measured)
        if map_pose is None:
            self.get_logger().warn(
                "No world->parrot1_map transform; cannot re-init after teleport "
                "(the sweep may drift). Check /mission/alignment.")
        else:
            self._publish_initial_pose(map_pose)
            self.get_logger().info(
                f"Teleported to world ({start_x:.1f}, {start_y:.1f}); re-init map pose "
                f"({map_pose.position.x:.1f}, {map_pose.position.y:.1f})")
            if not self._wait_for_map_pose(map_pose.position, 2.0):
                self.get_logger().warn(
                    "Map pose did not reach the teleport target; slam_toolbox re-init may "
                    "not have applied (is SLAM running?) - the sweep may drift")

        # 3. End-to-end check: the drone's MAP pose must sit on the ROI's first
        #    swath corner.  If map and world disagree the sweep is planned in a
        #    broken frame, so fail here instead of flying off into the trees.
        expected = self._sweep_start(
            (roi_min.x, roi_min.y), (roi_max.x, roi_max.y), float(args.swath_spacing))
        tf = self._lookup("parrot1_map", "parrot1_base_link")
        if expected is not None and tf is not None:
            err = math.hypot(tf.transform.translation.x - expected[0],
                             tf.transform.translation.y - expected[1])
            if err > 4.0:
                print(
                    "FAIL: drone map pose "
                    f"({tf.transform.translation.x:.1f}, {tf.transform.translation.y:.1f}) "
                    f"is {err:.1f} m from the ROI start ({expected[0]:.1f}, {expected[1]:.1f}). "
                    "world and parrot1_map are inconsistent - check /mission/alignment "
                    "and map->odom before sweeping.")
                return False
            self.get_logger().info(
                f"Map frame consistent: drone sits {err:.1f} m from the ROI start")
        return True

    def run(self, args) -> int:
        if not self.client.wait_for_server(timeout_sec=20.0):
            print("FAIL: /parrot1/sample_roi action server unavailable (launch the executor)")
            return 2

        frame, roi_min, roi_max = self.transform_box(args.roi, args.frame)
        if args.teleport:
            # Teleporting needs a healthy world<->map registration to re-init SLAM.
            if not self.wait_for_alignment(20.0):
                status = self.alignment_status or {}
                err = (status.get("errors") or {}).get("parrot1", {})
                print(
                    "FAIL: world<->parrot1_map alignment not ready "
                    f"(ready.parrot1 != true; error={err}). Localisation is unhealthy - "
                    "restart the stack and verify odom~=truth before sweeping.")
                return 2
            if not self.teleport_to_sweep_start(args, roi_min, roi_max):
                return 2
        elif not self.check_map_frame():
            return 2

        goal = SampleROI.Goal()
        goal.roi_origin = PoseStamped()
        goal.roi_origin.header.frame_id = frame
        goal.roi_origin.header.stamp = self.get_clock().now().to_msg()
        goal.roi_origin.pose.position.x = roi_min.x
        goal.roi_origin.pose.position.y = roi_min.y
        goal.roi_origin.pose.position.z = 0.0
        goal.roi_origin.pose.orientation.w = 1.0
        goal.roi_min = roi_min
        goal.roi_max = roi_max
        goal.search_clearance = float(args.search_clearance)
        goal.transit_clearance = float(args.transit_clearance)
        goal.swath_spacing = float(args.swath_spacing)

        def feedback_cb(msg) -> None:
            fb = msg.feedback
            print(f"  [{fb.stage}] progress {fb.progress:.0%} · trees {fb.trees_found}")

        print(
            f"Sending SampleROI goal: {frame} box "
            f"({roi_min.x:.1f}, {roi_min.y:.1f})-({roi_max.x:.1f}, {roi_max.y:.1f}), "
            f"clearance {goal.search_clearance:.1f} m"
        )
        goal_future = self.client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=30.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("FAIL: SampleROI goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        deadline = time.time() + args.timeout
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            print(f"FAIL: sampling did not finish within {args.timeout:.0f} s")
            return 1

        wrapped = result_future.result()
        message = wrapped.result.message if wrapped.result else ""
        print(f"SampleROI finished: {message}")

        # Let the latched tree map arrive.
        for _ in range(20):
            if self.tree_map is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        return 0


def score(node: RoiSamplingTester, gt: List[GroundTruthTree], args) -> int:
    returned = list(node.tree_map.trees) if node.tree_map is not None else []
    print(f"\nReturned trees: {len(returned)}   Ground truth in ROI: {len(gt)}")

    total = len(returned)
    loc_ok = 0
    type_ok = 0
    matches_per_gt = [0] * len(gt)
    rows = []

    for tree in returned:
        best_any = min(
            gt, key=lambda t: math.hypot(t.x - tree.location.x, t.y - tree.location.y)
        )
        err_any = math.hypot(best_any.x - tree.location.x, best_any.y - tree.location.y)
        same = [t for t in gt if t.tree_type == tree.tree_type]
        if same:
            best_same = min(
                same,
                key=lambda t: math.hypot(t.x - tree.location.x, t.y - tree.location.y),
            )
            err_same = math.hypot(
                best_same.x - tree.location.x, best_same.y - tree.location.y
            )
        else:
            err_same = float("inf")

        ok_loc = err_any <= args.error_threshold
        ok_type = err_same <= args.error_threshold
        loc_ok += 1 if ok_loc else 0
        type_ok += 1 if ok_type else 0
        if ok_loc:
            matches_per_gt[gt.index(best_any)] += 1
        rows.append(
            (tree.tree_type, tree.location.x, tree.location.y, err_any, ok_loc, ok_type)
        )

    recall = (
        sum(1 for count in matches_per_gt if count > 0) / len(gt) if gt else 0.0
    )
    precision = loc_ok / total if total else 0.0
    type_precision = type_ok / total if total else 0.0
    duplicates = sum(max(0, count - 1) for count in matches_per_gt)

    print("\nReturned tree                    err(m)  loc  type")
    for ttype, x, y, err, ok_loc, ok_type in rows:
        print(
            f"  {ttype:<10} ({x:7.1f}, {y:7.1f})  {err:5.2f}   "
            f"{'OK ' if ok_loc else 'BAD'}  {'OK' if ok_type else 'BAD'}"
        )

    print("\n==== Summary ====")
    print(f"Location precision: {loc_ok}/{total} = {precision:.2f}")
    print(f"Type precision:     {type_ok}/{total} = {type_precision:.2f}")
    print(f"ROI recall:         {sum(1 for c in matches_per_gt if c > 0)}/{len(gt)} = {recall:.2f}")
    print(f"Duplicate matches:  {duplicates}")

    # Name the trees that were never matched.  Without this a recall drop is
    # indistinguishable between "the sweep never flew past it", "YOLO missed
    # it" and "it sits outside the ROI and was filtered out on purpose".
    missed = [t for t, count in zip(gt, matches_per_gt) if count == 0]
    if missed:
        x_min, y_min, x_max, y_max = args.roi
        print(f"\nMissed ground-truth trees ({len(missed)} of {len(gt)}):")
        for t in missed:
            edge = min(t.x - x_min, x_max - t.x, t.y - y_min, y_max - t.y)
            if edge >= 0.0:
                where = f"{edge:.1f} m inside the box"
            else:
                where = f"{-edge:.1f} m OUTSIDE the box (filtered by roi_tree_margin)"
            print(f"  {t.tree_type:<10} ({t.x:7.1f}, {t.y:7.1f})  {where}")

    if total == 0:
        print("FAIL: no trees were returned")
        return 1
    if precision < args.min_location_precision:
        print(f"FAIL: location precision {precision:.2f} < {args.min_location_precision:.2f}")
        return 1
    if type_precision < args.min_type_precision:
        print(f"FAIL: type precision {type_precision:.2f} < {args.min_type_precision:.2f}")
        return 1
    if recall < args.min_recall:
        print(f"FAIL: recall {recall:.2f} < {args.min_recall:.2f}")
        return 1
    print("PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Parrot-only ROI sampling test.")
    parser.add_argument("--world", default="v2_forest")
    parser.add_argument("--world-file", default="")
    parser.add_argument("--frame", default="world", help="Frame the ROI box is given in.")
    parser.add_argument(
        "--roi",
        type=float,
        nargs=4,
        metavar=("X_MIN", "Y_MIN", "X_MAX", "Y_MAX"),
        default=list(DEFAULT_ROI),
        help="Preset ROI box in --frame (world metres).",
    )
    parser.add_argument("--roi-margin", type=float, default=0.0,
                        help="Ground-truth buffer around the ROI box (m). Keep this EQUAL to the "
                             "executor's roi_tree_margin, otherwise trees between the two boxes are "
                             "counted as ground truth but always filtered out of the result, which "
                             "silently caps recall.")
    parser.add_argument("--search-clearance", type=float, default=2.2)
    parser.add_argument("--transit-clearance", type=float, default=4.0)
    parser.add_argument("--swath-spacing", type=float, default=0.0,
                        help="<= 0 selects automatic spacing.")
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--error-threshold", type=float, default=4.0)
    parser.add_argument("--min-location-precision", type=float, default=0.9)
    parser.add_argument("--min-type-precision", type=float, default=0.8)
    parser.add_argument("--min-recall", type=float, default=0.7)
    parser.add_argument("--teleport", dest="teleport", action="store_true",
                        help="Teleport the drone to the nearest ROI corner instead of "
                             "flying there. Default is to fly to the nearest corner and "
                             "start the sweep from it.")
    parser.add_argument("--no-teleport", dest="teleport", action="store_false",
                        help="Deprecated alias: flying to the ROI is now the default.")
    parser.set_defaults(teleport=False)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    args.roi = tuple(args.roi)

    if args.world_file:
        world_file = Path(args.world_file)
    else:
        world_file = Path(get_package_share_directory("rs1_sim")) / "worlds" / f"{args.world}.sdf"
    if not world_file.is_file():
        print(f"FAIL: world file not found: {world_file}")
        return 2

    trees = parse_world_trees(world_file)
    gt = trees_in_box(trees, args.roi, args.roi_margin)
    print(f"Parsed {len(trees)} trees from {world_file}")
    print(f"ROI {args.roi} contains {len(gt)} ground-truth trees "
          f"(oak {sum(t.tree_type == 'OAK_TREE' for t in gt)}, "
          f"pine {sum(t.tree_type == 'PINE_TREE' for t in gt)})")
    if len(gt) < 3:
        print("FAIL: the preset ROI must contain at least 3 ground-truth trees")
        return 2

    rclpy.init()
    node = RoiSamplingTester(world_name=args.world)
    try:
        code = node.run(args)
        if code != 0:
            return code
        return score(node, gt, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
