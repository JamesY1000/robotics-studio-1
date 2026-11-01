#!/usr/bin/env python3
"""Measure lidar tree geometry to calibrate species verification.

Teleports the parrot to viewpoints FACING each ground-truth tree at several
altitudes (below canopy and above it) and records, for every tree/altitude/range
combination:

  * the contiguous lidar echo cluster width at the forward direction (m),
  * the number of beams in that cluster,
  * the cluster's range (m) and the drone's clearance above the surface below.

The drone is teleported to face the tree, so the tree is directly ahead; the
measurement works in the lidar frame and does NOT depend on the ``world`` TF
frame (which is a 2D map frame with z=0 in the navigation stack).

The CSV written to ``--output`` (default ``/tmp/tree_geometry.csv``) is the input
for choosing the pine/oak species discriminator (see the plan Phase 0).  It is a
diagnostic only - it does not call the perception services.

Run inside the sourced workspace with the sim + terrain following + lidar running:

    ros2 run rs1_perception measure_tree_geometry.py --altitudes 2.2 --ranges 8
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rs1_interfaces.srv import SetClearance

# Same cluster heuristics as tree_location_processing.py, kept here so the
# measurement tool stays standalone (no import of a node script).
CLUSTER_DEPTH_GAP = 0.3  # m; neighbour beams closer than front+gap belong to the trunk
MIN_BEAMS = 2
MAX_WIDTH_M = 20.0  # permissive for measurement (record canopies too)
RANGE_MIN = 0.2
RANGE_MAX = 40.0
EXPECTED_RANGE_TOL = 4.0  # m; the forward echo must be near the standoff distance
FORWARD_CONE = 0.26  # rad (~15 deg); search cone around the forward direction


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


class MeasureNode(Node):
    def __init__(self) -> None:
        super().__init__("measure_tree_geometry")
        self.scan_msg: Optional[LaserScan] = None
        self.down_msg: Optional[LaserScan] = None
        self.odom_z: Optional[float] = None
        self.create_subscription(LaserScan, "/parrot1/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/parrot1/downwards_scan", self._down_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/parrot1/odometry", self._odom_cb, qos_profile_sensor_data)
        self.clearance_client = self.create_client(SetClearance, "/parrot1/set_clearance")

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan_msg = msg

    def _down_cb(self, msg: LaserScan) -> None:
        self.down_msg = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_z = float(msg.pose.pose.position.z)

    def set_clearance(self, clearance: float, timeout: float = 5.0) -> bool:
        """Command the terrain follower to hold ``clearance`` metres."""
        if not self.clearance_client.service_is_ready():
            return False
        req = SetClearance.Request()
        req.clearance = float(clearance)
        fut = self.clearance_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        return res is not None and res.success

    def surface_clearance(self) -> Optional[float]:
        """Height above the surface directly below, from the downward lidar."""
        if self.down_msg is None:
            return None
        r = np.asarray(self.down_msg.ranges, dtype=np.float64)
        r = r[np.isfinite(r)]
        return None if r.size == 0 else float(np.median(r))

    def wait_for_altitude(self, min_z: float, timeout: float = 30.0) -> bool:
        """Spin until the drone holds at least ``min_z`` above the surface."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            z = self.surface_clearance()
            if z is not None and z >= min_z:
                return True
        return False

    def cluster_forward(self, expected_range: float):
        """Contiguous echo cluster at the drone's forward direction.

        The drone is teleported to face the tree, so the tree is directly ahead
        at ``expected_range``.  The cluster is seeded on the nearest valid echo
        in a forward cone around lidar angle 0 and grown circularly (the scan is
        360 deg, so a centred trunk straddles the 0/359 index seam).  Working in
        the lidar frame avoids any world/map-frame alignment mismatch.  Returns
        ``(width_m, n_beams, range_m)`` or ``None``.
        """
        scan = self.scan_msg
        if scan is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        n = ranges.size
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        norm = (angles + math.pi) % (2.0 * math.pi) - math.pi  # [-pi, pi]

        # Seed on the NEAREST valid echo in the forward cone (a centred trunk
        # may leave the exact angle-0 beam passing through foliage to background).
        forward = np.abs(norm) <= FORWARD_CONE
        cand = forward & np.isfinite(ranges) & (ranges >= RANGE_MIN) & (ranges <= RANGE_MAX)
        if not np.any(cand):
            return None
        seed = int(np.flatnonzero(cand)[np.argmin(ranges[cand])])
        seed_range = float(ranges[seed])
        # The forward echo must be the target tree, not background beyond it.
        if abs(seed_range - expected_range) > EXPECTED_RANGE_TOL:
            return None

        # Circular contiguous growth while neighbours stay at the trunk front.
        cluster_idx = [seed]
        i = seed
        while True:
            nxt = (i + 1) % n
            if nxt == seed or not (np.isfinite(ranges[nxt]) and ranges[nxt] <= seed_range + CLUSTER_DEPTH_GAP):
                break
            cluster_idx.append(nxt)
            i = nxt
        i = seed
        while True:
            prev = (i - 1) % n
            if prev == seed or not (np.isfinite(ranges[prev]) and ranges[prev] <= seed_range + CLUSTER_DEPTH_GAP):
                break
            cluster_idx.append(prev)
            i = prev

        n_beams = len(cluster_idx)
        if n_beams < MIN_BEAMS:
            return None
        cluster = ranges[cluster_idx]
        mean_range = float(np.median(cluster))
        width_rad = (n_beams - 1) * scan.angle_increment
        width_m = 2.0 * mean_range * math.sin(width_rad / 2.0)
        if width_m > MAX_WIDTH_M:
            return None
        return width_m, n_beams, mean_range


def main() -> int:
    ap = argparse.ArgumentParser(description="Measure lidar tree geometry for species calibration.")
    ap.add_argument("--world", default="v2_forest")
    ap.add_argument("--world-file", default="")
    ap.add_argument("--model-name", default="parrot1")
    ap.add_argument("--altitudes", type=float, nargs="+", default=[2.2, 8.0],
                    help="Drone heights above the tree base (m). 2.2 = below canopy, 8 = above.")
    ap.add_argument("--ranges", type=float, nargs="+", default=[8.0, 12.0],
                    help="Standoff distances from the tree (m).")
    ap.add_argument("--trees", type=int, default=40, help="Cap on trees to sample (0 = all).")
    ap.add_argument("--settle-time", type=float, default=1.5)
    ap.add_argument("--alt-timeout", type=float, default=30.0,
                    help="Max seconds to wait for the drone to climb to the requested altitude.")
    ap.add_argument("--output", default="/tmp/tree_geometry.csv")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

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

    rng = np.random.default_rng(args.seed)
    sampled = list(trees)
    rng.shuffle(sampled)
    if args.trees > 0:
        sampled = sampled[: args.trees]

    rclpy.init()
    node = MeasureNode()

    # Hold altitude via the terrain follower when it is running (the parrot has
    # no hover controller and base_link has no turnGravityOff, so a plain
    # set_pose falls to the ground).  Without it every row records ~0.2 m
    # clearance and the below/above-canopy sweep never happens.
    use_clearance = node.clearance_client.wait_for_service(timeout_sec=3.0)
    if use_clearance:
        print("Terrain following detected: holding altitude via /parrot1/set_clearance")
    else:
        print("WARNING: /parrot1/set_clearance unavailable; the drone will FALL after "
              "set_pose (launch rs1_navigation / parrot_navigation first). Measurements "
              "will be at ground level and useless for species calibration.")

    rows = 0
    try:
        for tree in sampled:
            # Stand off to one side so the tree is in front of the drone.
            sign = 1.0 if tree.x >= 0 else -1.0
            for alt in args.altitudes:
                for dist in args.ranges:
                    drone_x = max(-95.0, min(95.0, tree.x - sign * dist))
                    drone_y = max(-95.0, min(95.0, tree.y))
                    drone_z = tree.z + alt
                    yaw = 0.0 if sign > 0 else math.pi

                    if use_clearance and not node.set_clearance(alt):
                        print(f"  set_clearance({alt}) failed; skipping {tree.name}")
                        continue
                    if not set_pose(args.model_name, args.world, drone_x, drone_y, drone_z, yaw):
                        print(f"  set_pose failed for {tree.name}; skipping")
                        continue

                    if use_clearance:
                        # P gain 0.5 with max vertical speed 0.5 m/s: climbing to
                        # `alt` from the ground can take ~2*alt seconds.
                        if not node.wait_for_altitude(alt * 0.7, timeout=args.alt_timeout):
                            print(f"  {tree.name}: did not reach ~{alt:.1f} m in time; recording anyway")
                    else:
                        time.sleep(args.settle_time)

                    # Spin long enough for the 3 Hz forward lidar to publish a
                    # FRESH scan after the teleport (otherwise the cluster is
                    # measured from the previous tree's position -> duplicate).
                    fresh_deadline = time.time() + 0.7
                    while time.time() < fresh_deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)

                    # Height above the surface below (varies with the hilly
                    # terrain; the odometry z is the real world height).
                    clearance = node.surface_clearance()
                    drone_z = node.odom_z
                    # Lidar sits 0.2 m above base_link; this is the height of
                    # the horizontal ring ABOVE THE TREE BASE - the value that
                    # actually determines which part of the trunk/cone is sliced.
                    ring_height = (drone_z + 0.2) - tree.z if drone_z is not None else None

                    # The drone faces the tree, so the trunk/cone is directly
                    # ahead at ~standoff distance.  Scan the forward sector.
                    cluster = node.cluster_forward(dist)
                    width_m = cluster[0] if cluster else -1.0
                    n_beams = cluster[1] if cluster else 0
                    range_m = cluster[2] if cluster else -1.0
                    rows += 1
                    print(f"  {tree.name:20s} {tree.tree_type:10s} alt={alt:4.1f} "
                          f"dist={dist:4.1f} clear={clearance if clearance is not None else -1:5.2f} "
                          f"ring_h={ring_height if ring_height is not None else -1:5.2f} "
                          f"-> width={width_m:.2f} m, beams={n_beams}, range={range_m:.1f} m")
                    with open(args.output, "a", newline="") as fh:
                        w = csv.writer(fh)
                        if fh.tell() == 0:
                            w.writerow(["name", "type", "altitude_m", "standoff_m", "surface_clearance_m",
                                        "ring_height_above_tree_base_m",
                                        "cluster_width_m", "n_beams", "cluster_range_m"])
                        w.writerow([tree.name, tree.tree_type, alt, dist,
                                    clearance if clearance is not None else -1.0,
                                    ring_height if ring_height is not None else -1.0,
                                    width_m, n_beams, range_m])
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"\nWrote {rows} measurement(s) to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
