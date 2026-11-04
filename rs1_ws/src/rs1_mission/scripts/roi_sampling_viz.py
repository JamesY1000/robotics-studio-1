#!/usr/bin/env python3
"""RViz visualisation for the ROI sampling phase.

Publishes latched MarkerArrays (frame ``world``) so RViz can show:

* the ROI box bounds                  -> ``/roi_sampling/roi``
* ground-truth trees inside the ROI   -> ``/roi_sampling/ground_truth``
* trees found by the parrot           -> ``/roi_sampling/found_trees``

Found trees come from the parrot's latched ``/parrot1/tree_map`` (``TreeMap``)
topic; ground truth is parsed from the world SDF. Pair it with the RViz config:

    ros2 run rs1_mission roi_sampling_viz.py --roi 62 -88 92 -58
    rviz2 -d $(ros2 pkg prefix rs1_mission)/share/rs1_mission/config/roi_sampling.rviz

or start both at once:

    ros2 launch rs1_mission roi_sampling_viz.launch.py
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from rs1_interfaces.msg import TreeMap

DEFAULT_ROI = (62.0, -88.0, 92.0, -58.0)
# Everything drawn here is Gazebo ground truth: the ROI corners come from the
# command line (world metres) and the ground-truth trees are parsed from the world
# SDF, while /parrot1/tree_map carries the same world coordinates (perception
# anchors on the raw Gazebo odometry).  This used to be parrot1_odom because the
# EKF then relayed the exact Gazebo pose and the two frames coincided; with a
# twist-only EKF they do not, so the markers are published in `world` and RViz
# transforms them into whatever fixed frame it uses.
FRAME = "world"

TYPE_COLORS = {
    "OAK_TREE": (0.85, 0.45, 0.10),
    "PINE_TREE": (0.15, 0.75, 0.25),
}
UNKNOWN_COLOR = (0.90, 0.10, 0.90)
GT_COLOR = (0.55, 0.55, 0.55)

ROI_TOPIC = "/roi_sampling/roi"
GT_TOPIC = "/roi_sampling/ground_truth"
FOUND_TOPIC = "/roi_sampling/found_trees"

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


@dataclass
class GroundTruthTree:
    name: str
    tree_type: str
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


def trees_in_box(trees, roi, margin):
    x_min, y_min, x_max, y_max = roi
    return [
        t
        for t in trees
        if x_min - margin <= t.x <= x_max + margin
        and y_min - margin <= t.y <= y_max + margin
    ]


def sphere(marker_id: int, ns: str, x: float, y: float, z: float,
           color, scale: float) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = ns
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b = color
    m.color.a = 0.9
    return m


def label(marker_id: int, ns: str, x: float, y: float, z: float,
          text: str, color) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = ns
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.w = 1.0
    m.scale.z = 0.9
    m.color.r, m.color.g, m.color.b = color
    m.color.a = 1.0
    m.text = text
    return m


def delete_all(ns: str) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.ns = ns
    m.id = 0
    m.action = Marker.DELETEALL
    return m


class RoiSamplingViz(Node):
    def __init__(self, args, ground_truth: List[GroundTruthTree]) -> None:
        super().__init__("roi_sampling_viz")
        self.args = args
        self.ground_truth = ground_truth

        self.roi_pub = self.create_publisher(MarkerArray, ROI_TOPIC, LATCHED)
        self.gt_pub = self.create_publisher(MarkerArray, GT_TOPIC, LATCHED)
        self.found_pub = self.create_publisher(MarkerArray, FOUND_TOPIC, LATCHED)

        self.create_subscription(TreeMap, "/parrot1/tree_map", self._on_tree_map, LATCHED)

        self.publish_roi()
        self.publish_ground_truth()

    def publish_roi(self) -> None:
        x_min, y_min, x_max, y_max = self.args.roi
        z = self.args.roi_z
        outline = Marker()
        outline.header.frame_id = FRAME
        outline.ns = "roi_bounds"
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.4
        outline.color.r = 0.0
        outline.color.g = 1.0
        outline.color.b = 1.0
        outline.color.a = 0.9
        for cx, cy in [
            (x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max), (x_min, y_min),
        ]:
            outline.points.append(Point(x=cx, y=cy, z=z))

        text = label(
            1, "roi_label", (x_min + x_max) / 2, (y_min + y_max) / 2, z + 2.0,
            f"ROI {x_min:.0f},{y_min:.0f} -> {x_max:.0f},{y_max:.0f}",
            (0.0, 1.0, 1.0),
        )

        self.roi_pub.publish(MarkerArray(markers=[outline, text]))

    def publish_ground_truth(self) -> None:
        markers = [delete_all("ground_truth")]
        for i, tree in enumerate(self.ground_truth):
            markers.append(sphere(i, "ground_truth", tree.x, tree.y, tree.z + 1.0,
                                  GT_COLOR, 1.0))
            markers.append(label(i, "ground_truth_label", tree.x, tree.y, tree.z + 3.0,
                                 f"GT {tree.tree_type}", GT_COLOR))
        self.gt_pub.publish(MarkerArray(markers=markers))

    def _on_tree_map(self, msg: TreeMap) -> None:
        markers = [delete_all("found_trees")]
        for i, tree in enumerate(msg.trees):
            color = TYPE_COLORS.get(tree.tree_type, UNKNOWN_COLOR)
            x, y, z = tree.location.x, tree.location.y, tree.location.z
            markers.append(sphere(i, "found_trees", x, y, z + 1.0, color, 1.1))
            markers.append(label(i, "found_labels", x, y, z + 3.5,
                                 f"{tree.tree_type} x{tree.observations}", color))
        self.found_pub.publish(MarkerArray(markers=markers))
        self.get_logger().info(f"Found trees: {len(msg.trees)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="RViz markers for ROI sampling.")
    parser.add_argument("--world", default="v2_forest")
    parser.add_argument("--world-file", default="")
    parser.add_argument("--roi", type=float, nargs=4,
                        metavar=("X_MIN", "Y_MIN", "X_MAX", "Y_MAX"), default=list(DEFAULT_ROI))
    parser.add_argument("--roi-margin", type=float, default=2.0,
                        help="Ground-truth buffer around the ROI box (m).")
    parser.add_argument("--roi-z", type=float, default=0.0, help="ROI marker plane height (m).")
    parser.add_argument("--show-ground-truth", action="store_true",
                        help="Publish the ground-truth markers (default on).")
    parser.add_argument("--no-ground-truth", action="store_true")
    parser.add_argument("--rviz", action="store_true", help="Start rviz2 with the bundled config.")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    args.roi = tuple(args.roi)

    if args.world_file:
        world_file = Path(args.world_file)
    else:
        world_file = Path(get_package_share_directory("rs1_sim")) / "worlds" / f"{args.world}.sdf"
    if not world_file.is_file():
        print(f"World file not found: {world_file}")
        return 2

    trees = parse_world_trees(world_file)
    ground_truth = trees_in_box(trees, args.roi, args.roi_margin)
    print(f"Parsed {len(trees)} trees; ROI {args.roi} contains {len(ground_truth)} ground truth")

    if args.rviz:
        config = Path(get_package_share_directory("rs1_mission")) / "config" / "roi_sampling.rviz"
        subprocess.Popen(["rviz2", "-d", str(config)])

    rclpy.init()
    node = RoiSamplingViz(args, [] if args.no_ground_truth else ground_truth)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
