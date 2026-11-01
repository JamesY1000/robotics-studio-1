#!/usr/bin/env python3
"""Interactively build a map of trees while you fly the Parrot by hand.

Fly with ``teleop_twist_keyboard`` in one terminal and run this in another. Each
time you press the snapshot key (``p`` by default) it:

1. calls ``/yolo/detect`` (YOLO on the current RGB frame),
2. calls ``/yolo/process_tree_locations`` (depth + lidar -> world tree bases),
3. merges the found trees into a persistent map (a tree seen again from another
   angle is merged into the existing entry, not duplicated),
4. publishes the whole map on ``/tree_locations/map`` as RViz markers.

Keys (these only apply to THIS terminal)::

    p   take a snapshot and add the trees to the map
    c   clear the map (press s afterwards so it stays cleared)
    s   save the map to the map file (CSV)
    l   list the map in this terminal
    h   show the help
    q   quit (saves automatically)

Typical use::

    # terminal 1 - simulation
    ros2 launch rs1_sim rs1_sim.launch.py husky:=false rviz:=false slam:=false nav2:=false

    # terminal 2 - perception services
    ros2 launch rs1_perception tree_identification.launch.py

    # terminal 3 - RViz on the map topic
    rviz2 -d $(ros2 pkg prefix rs1_perception)/share/rs1_perception/config/tree_locations.rviz

    # terminal 4 - fly around (keep this on ONE line - it is meant to be
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/parrot1/cmd_vel -p speed:=0.3 -p turn:=0.5

    # terminal 5 - press p to snapshot trees into the map, while still flying
    ros2 run rs1_perception map_tree_locations.py

Fly and snapshot at the same time: the keys you type here never reach the teleop
terminal, and Gazebo's VelocityControl holds the last commanded velocity, so the
drone carries on moving while you tap the snapshot key. The snapshot key is ``p`` by default
because teleop_twist_keyboard binds ``i o j l u m , . t b`` (motion) and
``q z w x e c`` (speed), and it treats ANY other key - ``k`` and space included -
as a stop (it zeroes the velocity). So avoid those keys here:
``--snapshot-key p`` is safe, ``--snapshot-key i,j,k,l`` is not.

This tool needs an interactive terminal, so run it with ``ros2 run`` rather than
from a launch file.

If teleop runs but the drone will not move, its ``cmd_vel`` remap did not reach
the node: teleop then publishes to ``/cmd_vel`` while the bridge only listens on
``/parrot1/cmd_vel``. Check with ``ros2 topic info -v /parrot1/cmd_vel`` - the
publisher must be ``/teleop_twist_keyboard``, not the bare ``/cmd_vel`` topic.
Typical causes: the command was split across lines with a backslash continuation
that bash did not honour (see the fixed one-line command above), or the
``--ros-args`` part was dropped when pasting.

RViz shows the map on ``/tree_locations/map`` plus a grey marker at every true tree
position (read from the world SDF), so a missed tree appears as grey with no
coloured marker next to it. The grey ones are on by default - press ``g`` to hide
them.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import select
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.duration import Duration
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
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rs1_interfaces.srv import DetectYolo, ProcessTreeLocations

TYPE_COLORS = {
    "OAK_TREE": (0.85, 0.45, 0.10),   # orange
    "PINE_TREE": (0.15, 0.75, 0.25),  # green
}
UNKNOWN_COLOR = (0.90, 0.10, 0.90)    # magenta
GROUND_TRUTH_COLOR = (0.60, 0.60, 0.60)
TRAIL_COLOR = (0.30, 0.70, 1.00)      # blue

MAP_TOPIC = "/tree_locations/map"
NS_TREES = "map_trees"
NS_LABELS = "map_labels"
NS_LATEST = "map_latest_snapshot"
NS_GT = "ground_truth_trees"
NS_TRAIL = "drone_trail"

LATCHED_QOS = QoSProfile(
    depth=5,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

MAP_CSV_HEADER = "tree_type,x,y,z,observations"

# Keys that teleop_twist_keyboard consumes (motion + speed). Pressing one of
# these here is almost certainly a mistake, so we print a nudge.
TELEOP_KEYS = set("ijklum,tbqzwxec") | {"<", ">"}


@dataclass
class MappedTree:
    tree_type: str
    x: float
    y: float
    z: float
    observations: int = 1


@dataclass
class GroundTruthTree:
    name: str
    tree_type: str
    x: float
    y: float
    z: float


def parse_world_trees(world_file: Path) -> List[GroundTruthTree]:
    """Ground-truth trees from the world SDF (used by --show-ground-truth)."""
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


class RawKeyReader:
    """Single-keypress reader (cbreak mode), while keeping the node spinning."""

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.enabled = sys.stdin.isatty()
        self._old = None

    def __enter__(self) -> "RawKeyReader":
        if self.enabled:
            self._old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc) -> None:
        if self.enabled and self._old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)

    def read(self, node, timeout: float = 0.1) -> Optional[str]:
        """One keypress, "" if none was pressed, or None on EOF."""
        rclpy.spin_once(node, timeout_sec=timeout)
        if not self.enabled:
            time.sleep(timeout)
            return ""
        if select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(self.fd, 1)
            if data == b"":
                return None
            return data.decode(errors="ignore")
        return ""


class TreeMapBuilder(Node):
    def __init__(self, args, ground_truth: List[GroundTruthTree]) -> None:
        super().__init__("tree_map_builder")
        self.args = args
        self.ground_truth = ground_truth

        self.detect_client = self.create_client(DetectYolo, "/yolo/detect")
        self.process_client = self.create_client(
            ProcessTreeLocations, "/yolo/process_tree_locations"
        )
        self.marker_pub = self.create_publisher(MarkerArray, MAP_TOPIC, LATCHED_QOS)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map: List[MappedTree] = []
        self.last_added: List[MappedTree] = []
        self.trail: List[tuple] = []
        self.snapshots = 0
        self._last_bounds_warn = 0.0

        if self.args.load_map:
            self.load_map()

    # ------------------------------------------------------------- services
    def _call(self, client, request, timeout: float = 60.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                f"Service {client.srv_name} unavailable - is "
                f"`ros2 launch rs1_perception tree_identification.launch.py` running?"
            )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"Call to {client.srv_name} failed")
        return future.result()

    # ------------------------------------------------------------ map merge
    def _merge(self, tree_type: str, x: float, y: float, z: float) -> tuple:
        """Add or update a tree; returns (entry, is_new)."""
        best = None
        best_d = self.args.merge_distance
        for t in self.map:
            if t.tree_type != tree_type:
                continue
            d = math.hypot(t.x - x, t.y - y)
            if d <= best_d:
                best, best_d = t, d

        if best is None:
            entry = MappedTree(tree_type, x, y, z, 1)
            self.map.append(entry)
            return entry, True

        # Running average, weighted by how many times we have seen it.
        n = best.observations
        best.x = (best.x * n + x) / (n + 1)
        best.y = (best.y * n + y) / (n + 1)
        best.z = (best.z * n + z) / (n + 1)
        best.observations = n + 1
        return best, False

    def snapshot(self) -> None:
        """Detect + process once, then merge the trees into the map."""
        req = DetectYolo.Request()
        req.start_yolo = True
        detections = self._call(self.detect_client, req).detections

        preq = ProcessTreeLocations.Request()
        preq.start_lidar = True
        preq.detections = detections
        locations = self._call(self.process_client, preq).tree_data

        self.snapshots += 1
        self.last_added = []
        new = updated = 0
        for loc in locations:
            entry, is_new = self._merge(
                loc.tree_type, loc.location.x, loc.location.y, loc.location.z
            )
            if not any(e is entry for e in self.last_added):
                self.last_added.append(entry)
            new += 1 if is_new else 0
            updated += 0 if is_new else 1

        self.publish_markers()
        self.get_logger().info(
            f"[snapshot {self.snapshots}] {len(detections.bounding_boxes)} boxes -> "
            f"{len(locations)} trees ({new} new, {updated} updated) | "
            f"map now has {len(self.map)} tree(s)"
        )

    # -------------------------------------------------------------- markers
    @staticmethod
    def _marker(ns: str, marker_id: int, action: int = Marker.ADD) -> Marker:
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = marker_id
        m.action = action
        return m

    def _type_counts(self) -> dict:
        counts = {}
        for t in self.map:
            counts[t.tree_type] = counts.get(t.tree_type, 0) + 1
        return counts

    def _build_markers(self) -> MarkerArray:
        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for ns in (NS_TREES, NS_LABELS, NS_LATEST, NS_GT, NS_TRAIL):
            array.markers.append(self._marker(ns, 0, Marker.DELETEALL))

        # The accumulated map.
        for i, t in enumerate(self.map):
            r, g, b = TYPE_COLORS.get(t.tree_type, UNKNOWN_COLOR)
            sphere = self._marker(NS_TREES, i)
            sphere.header.stamp = now
            sphere.type = Marker.SPHERE
            sphere.pose.position.x, sphere.pose.position.y = t.x, t.y
            sphere.pose.position.z = t.z + 0.75
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.5
            sphere.color.r, sphere.color.g, sphere.color.b = r, g, b
            sphere.color.a = 0.95
            array.markers.append(sphere)

            label = self._marker(NS_LABELS, i)
            label.header.stamp = now
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y = t.x, t.y
            label.pose.position.z = t.z + 2.4
            label.text = f"{t.tree_type} x{t.observations}\n({t.x:.1f}, {t.y:.1f})"
            label.scale.z = 0.9
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            array.markers.append(label)

        # Highlight whatever the most recent snapshot touched.
        for i, t in enumerate(self.last_added):
            hl = self._marker(NS_LATEST, i)
            hl.header.stamp = now
            hl.type = Marker.SPHERE
            hl.pose.position.x, hl.pose.position.y = t.x, t.y
            hl.pose.position.z = t.z + 0.75
            hl.pose.orientation.w = 1.0
            hl.scale.x = hl.scale.y = hl.scale.z = 2.6
            hl.color.r = hl.color.g = hl.color.b = 1.0
            hl.color.a = 0.25
            array.markers.append(hl)

        if self.args.show_ground_truth:
            for i, gt in enumerate(self.ground_truth):
                m = self._marker(NS_GT, i)
                m.header.stamp = now
                m.type = Marker.SPHERE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = gt.x, gt.y, gt.z + 0.6
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 1.2
                m.color.r, m.color.g, m.color.b = GROUND_TRUTH_COLOR
                m.color.a = 0.4
                array.markers.append(m)

        if self.args.trail and len(self.trail) >= 2:
            line = self._marker(NS_TRAIL, 0)
            line.header.stamp = now
            line.type = Marker.LINE_STRIP
            line.pose.orientation.w = 1.0
            line.scale.x = 0.3
            line.color.r, line.color.g, line.color.b = TRAIL_COLOR
            line.color.a = 0.9
            for x, y, z in self.trail:
                line.points.append(Point(x=x, y=y, z=z))
            array.markers.append(line)

        return array

    def publish_markers(self) -> None:
        self.marker_pub.publish(self._build_markers())

    # ------------------------------------------------------------- drone pos
    def update_trail(self) -> bool:
        """Append the drone's position if it moved; True if the trail changed."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "world", "parrot1_base_link", Time(), timeout=Duration(seconds=0.2)
            )
        except Exception:  # noqa: BLE001
            return False
        t = tf.transform.translation
        pos = (t.x, t.y, t.z)

        # VelocityControl holds the last commanded velocity, so the drone keeps
        # flying after teleop stops. Warn if it has left the forest.
        limit = self.args.forest_limit
        if abs(pos[0]) > limit or abs(pos[1]) > limit:
            now = time.time()
            if now - self._last_bounds_warn > 10.0:
                self._last_bounds_warn = now
                print(f"\nWARNING: drone at ({pos[0]:.0f}, {pos[1]:.0f}) is outside the forest "
                      f"(+/-{limit:.0f} m). Press 'k' in the TELEOP window to stop it, then "
                      f"teleport it back (see the docstring).")

        if self.trail and math.dist(self.trail[-1], pos) < self.args.trail_step:
            return False
        self.trail.append(pos)
        return True

    # ------------------------------------------------------------ map files
    def load_map(self) -> None:
        path = Path(self.args.map_file)
        if not path.is_file():
            return
        loaded = 0
        for line in path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("tree_type"):
                continue
            parts = line.split(",")
            if len(parts) < 5:
                continue
            try:
                self.map.append(
                    MappedTree(parts[0], float(parts[1]), float(parts[2]), float(parts[3]), int(parts[4]))
                )
                loaded += 1
            except ValueError:
                continue
        if loaded:
            print(f"Loaded {loaded} tree(s) from {path}")

    def save_map(self) -> None:
        path = Path(self.args.map_file)
        lines = [MAP_CSV_HEADER]
        for t in self.map:
            lines.append(f"{t.tree_type},{t.x:.3f},{t.y:.3f},{t.z:.3f},{t.observations}")
        path.write_text("\n".join(lines) + "\n")
        print(f"Saved {len(self.map)} tree(s) to {path}")

    def describe(self) -> None:
        counts = self._type_counts()
        summary = ", ".join(f"{k}: {v}" for k, v in sorted(counts.items())) or "empty"
        print(f"\nMap: {len(self.map)} tree(s) [{summary}]  (after {self.snapshots} snapshot(s))")
        for i, t in enumerate(self.map):
            print(f"  [{i:3d}] {t.tree_type:9} ({t.x:7.2f}, {t.y:7.2f}, {t.z:5.2f})  seen x{t.observations}")


HELP_TEXT = """
Keys (this terminal only - teleop keeps flying meanwhile)
  p   take a snapshot and add the detected trees to the map
  c   clear the map (press 's' afterwards so it stays cleared)
  s   save the map to the CSV file
  l   list the trees in the map
  g   toggle the grey ground-truth markers (on by default)
  h   show this help
  q   quit (saves automatically)

Avoid teleop's keys if you are flying at the same time: i o j l u m , . t b
(motion) and q z w x e c (speed) - teleop treats any other key as a stop.
"""


def main() -> int:
    ap = argparse.ArgumentParser(description="Build a tree map from YOLO snapshots taken on keypress.")
    ap.add_argument("--snapshot-key", default="p",
                    help="Key that takes a snapshot; comma separated for several "
                         "(default: p). Avoid teleop_twist_keyboard's keys "
                         "(i o j l u m , . t b q z w x e c), which teleop treats as "
                         "motion/speed, and every other key there is a stop.")
    ap.add_argument("--map-file", default="/tmp/tree_map.csv", help="CSV file to save/load the map.")
    ap.add_argument("--load-map", action=argparse.BooleanOptionalAction, default=True,
                    help="Load --map-file at startup if it exists.")
    ap.add_argument("--save-on-exit", action=argparse.BooleanOptionalAction, default=True,
                    help="Save the map when quitting.")
    ap.add_argument("--merge-distance", type=float, default=3.0,
                    help="Trees of the same type within this distance (m) are merged.")
    ap.add_argument("--trail", action=argparse.BooleanOptionalAction, default=True,
                    help="Publish the drone's flown path as a line.")
    ap.add_argument("--trail-step", type=float, default=1.0,
                    help="Minimum distance (m) between trail points.")
    ap.add_argument("--forest-limit", type=float, default=98.0,
                    help="Warn if the drone goes beyond +/- this in x or y (the forest walls).")
    ap.add_argument("--show-ground-truth", action=argparse.BooleanOptionalAction, default=True,
                    help="Draw a grey marker at every true tree position from the world SDF "
                         "(default: on - press 'g' to toggle).")
    ap.add_argument("--world", default="v2_forest")
    ap.add_argument("--world-file", default="")
    ap.add_argument("--rviz", action=argparse.BooleanOptionalAction, default=False,
                    help="Also launch RViz with config/tree_locations.rviz.")
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    if not sys.stdin.isatty():
        print("ERROR: this tool needs an interactive terminal (press keys here).\n"
              "       Run it with `ros2 run rs1_perception map_tree_locations.py`\n"
              "       rather than from a launch file.")
        return 2

    if args.world_file:
        world_file = Path(args.world_file)
    else:
        world_file = Path(get_package_share_directory("rs1_sim")) / "worlds" / f"{args.world}.sdf"
    ground_truth: List[GroundTruthTree] = []
    if world_file.is_file():
        ground_truth = parse_world_trees(world_file)
        print(f"Parsed {len(ground_truth)} ground-truth trees from {world_file}")
    else:
        print(f"World file not found ({world_file}); ground-truth hints disabled")

    rviz_proc = None
    if args.rviz:
        config = Path(get_package_share_directory("rs1_perception")) / "config" / "tree_locations.rviz"
        print(f"Launching RViz with {config}")
        rviz_proc = subprocess.Popen(["rviz2", "-d", str(config)])

    rclpy.init()
    node = TreeMapBuilder(args, ground_truth)

    snapshot_keys = {k.strip().lower() for k in args.snapshot_key.split(",") if k.strip()}

    print(f"\nBuilding a tree map. Markers on {MAP_TOPIC} (fixed frame 'world').")
    print(f"Press '{'/'.join(sorted(snapshot_keys))}' to snapshot, 'h' for help, 'q' to quit.")
    print("Keep flying - teleop's keys stay in its own terminal.")

    try:
        with RawKeyReader() as keys:
            node.publish_markers()
            while rclpy.ok():
                if node.update_trail():
                    node.publish_markers()
                key = keys.read(node)
                if key is None:            # stdin closed
                    break
                key = key.lower()
                if key in ("q", "\x03", "\x04"):
                    break
                if key in snapshot_keys:
                    try:
                        node.snapshot()
                    except RuntimeError as ex:
                        print(f"ERROR: {ex}")
                elif key == "c":
                    node.map.clear()
                    node.last_added = []
                    node.publish_markers()
                    print("Map cleared (markers deleted). Only the in-memory map was cleared - "
                          f"press 's' to overwrite {node.args.map_file} with the empty map, "
                          "otherwise the old trees are reloaded next time.")
                elif key == "s":
                    node.save_map()
                elif key == "l":
                    node.describe()
                elif key == "g":
                    args.show_ground_truth = not args.show_ground_truth
                    node.publish_markers()
                    print(f"Ground-truth markers: {'on' if args.show_ground_truth else 'off'}")
                elif key == "h":
                    print(HELP_TEXT)
                elif key in TELEOP_KEYS:
                    key_list = "/".join(sorted(snapshot_keys))
                    print(f"'{key}' is a teleop key (it only affects the teleop terminal).\n"
                          f"  Press '{key_list}' here to snapshot, 'h' for help.")
    except KeyboardInterrupt:
        pass
    finally:
        if args.save_on_exit and node.map:
            node.save_map()
        node.destroy_node()
        rclpy.shutdown()
        if rviz_proc is not None:
            rviz_proc.terminate()

    print(f"\nDone. {len(node.map)} tree(s) in the map.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
