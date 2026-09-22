#!/usr/bin/env python3
"""Visual check for the YOLO tree-detection -> TreeLocation pipeline.

Each cycle this node:

1. calls ``/yolo/detect`` (YOLO on the latest RGB frame),
2. feeds the detections into ``/yolo/process_tree_locations`` (depth + lidar ->
   world-frame tree bases),
3. publishes a ``visualization_msgs/MarkerArray`` on ``/tree_locations/markers``
   with one coloured marker per identified tree - a different colour per tree
   type - plus a text label.

Open RViz with the fixed frame ``world`` and a MarkerArray display on
``/tree_locations/markers`` (see ``config/tree_locations.rviz``) to see the
markers land on the trees. Use ``--rviz`` to have this script launch RViz for
you.

Two extra ways to check the result against reality:

* ``--gazebo`` - spawns a translucent coloured pole in the Gazebo world itself at
  every detected tree base, so you can see the marker sitting next to the real
  tree in the simulator's 3D view.
* ``--show-ground-truth`` - publishes a plain grey marker at *every* tree in the
  world SDF, so RViz shows the full forest layout next to the detections.

Typical use::

    # terminal 1
    ros2 launch rs1_sim rs1_sim.launch.py husky:=false rviz:=false
    # terminal 2
    ros2 launch rs1_perception tree_identification.launch.py
    # terminal 3 - fly a tour of the forest, updating the markers
    ros2 run rs1_perception visualise_tree_locations.py --tour --rviz

Without ``--tour`` the markers are simply refreshed at the drone's current pose
every ``--interval`` seconds, so you can teleport/drive the drone yourself and
watch the detected trees update.

Press-Enter-to-continue (``--step``) instead of a timer::

    # step through the tour by hand: Enter = next stop, 'q' = quit
    ros2 run rs1_perception visualise_tree_locations.py --tour --step

    # no tour: re-run detection at the current pose each time you press Enter
    ros2 run rs1_perception visualise_tree_locations.py --step

``--step`` needs an interactive terminal, so run it with ``ros2 run`` rather than
from a launch file (it falls back to a short pause if stdin is not a terminal).
"""

from __future__ import annotations

import argparse
import math
import random
import re
import select
import subprocess
import sys
import time
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
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rs1_interfaces.srv import DetectYolo, ProcessTreeLocations

# One colour per tree type (RGB, 0..1). Unknown labels get magenta.
TYPE_COLORS = {
    "OAK_TREE": (0.85, 0.45, 0.10),   # orange
    "PINE_TREE": (0.15, 0.75, 0.25),  # green
}
UNKNOWN_COLOR = (0.90, 0.10, 0.90)    # magenta
GROUND_TRUTH_COLOR = (0.60, 0.60, 0.60)  # grey

MARKER_TOPIC = "/tree_locations/markers"
NS_TREES = "detected_trees"
NS_LABELS = "detected_tree_labels"
NS_GT = "ground_truth_trees"

# Markers are latched so a late-joining RViz still shows the last result.
LATCHED_QOS = QoSProfile(
    depth=5,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

# ---- Gazebo markers -------------------------------------------------------
# Optional: spawn a translucent coloured pole in the Gazebo world at each
# detected tree, so the result can be checked against the actual trees in 3D.
GAZEBO_MARKER_DIR = Path("/tmp/tree_location_markers")
GAZEBO_MARKER_RADIUS = 0.4
GAZEBO_MARKER_LENGTH = 5.0
GAZEBO_MARKER_MAX = 24  # tree_marker_0 .. tree_marker_23
GAZEBO_MARKER_TEMPLATE = """<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <transparency>0.5</transparency>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _run_ign_service(service: str, reqtype: str, reptype: str, request: str, timeout: int = 10000) -> bool:
    """Call a Gazebo (Fortress) service (mirrors sample_generator.helpers)."""
    cmd = [
        "ign", "service", "-s", service,
        "--reqtype", reqtype, "--reptype", reptype,
        "--timeout", str(timeout), "--req", request,
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True)
    except FileNotFoundError:
        return False
    return res.returncode == 0 and "true" in (res.stdout + res.stderr).lower()


def gazebo_remove_model(world: str, name: str) -> bool:
    return _run_ign_service(
        f"/world/{world}/remove", "ignition.msgs.Entity", "ignition.msgs.Boolean",
        f'name: "{name}", type: MODEL', timeout=2000,
    )


def gazebo_spawn_model(world: str, name: str, sdf_file: str, x: float, y: float, z: float) -> bool:
    request = (
        f'sdf_filename: "{sdf_file}", name: "{name}", '
        f"pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ w: 1.0 }} }}"
    )
    return _run_ign_service(
        f"/world/{world}/create", "ignition.msgs.EntityFactory",
        "ignition.msgs.Boolean", request, timeout=10000,
    )


def write_marker_templates() -> dict:
    """Write one marker SDF per tree type; returns ``{type_key: sdf_path}``."""
    GAZEBO_MARKER_DIR.mkdir(parents=True, exist_ok=True)
    files = {}
    colours = dict(TYPE_COLORS)
    colours["UNKNOWN"] = UNKNOWN_COLOR
    for key, (r, g, b) in colours.items():
        path = GAZEBO_MARKER_DIR / f"marker_{key.lower()}.sdf"
        path.write_text(
            GAZEBO_MARKER_TEMPLATE.format(
                name=f"marker_{key.lower()}",
                radius=GAZEBO_MARKER_RADIUS,
                length=GAZEBO_MARKER_LENGTH,
                r=r, g=g, b=b,
            )
        )
        files[key] = str(path)
    return files


@dataclass
class GroundTruthTree:
    name: str
    tree_type: str  # OAK_TREE | PINE_TREE
    x: float
    y: float
    z: float


def parse_world_trees(world_file: Path) -> List[GroundTruthTree]:
    """Ground-truth trees from the world SDF (used for labels/hints only)."""
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
    """Teleport a Gazebo model (Fortress ``ign service``)."""
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


def choose_viewpoints(trees: List[GroundTruthTree], count: int, min_sep: float) -> List[GroundTruthTree]:
    """Pick well-separated trees to fly to."""
    ordered = list(trees)
    random.shuffle(ordered)
    chosen: List[GroundTruthTree] = []
    for t in ordered:
        if all(math.hypot(t.x - c.x, t.y - c.y) >= min_sep for c in chosen):
            chosen.append(t)
        if len(chosen) >= count:
            break
    return chosen


def wait_for_user_input(node, prompt: str, reminder_every: float = 30.0) -> str:
    """Block until the user presses Enter, while keeping the node spinning.

    Returns:
        ``"next"``     - Enter pressed (continue)
        ``"quit"``     - the user typed q/quit/exit
        ``"eof"``      - stdin was closed
        ``"no-tty"``   - not an interactive terminal (e.g. started from a launch
                         file); a short pause is taken instead of blocking
        ``"shutdown"`` - rclpy stopped
    """
    if not sys.stdin.isatty():
        print(prompt + " [stdin is not a terminal - continuing automatically]")
        node.spin_for(2.0)
        return "no-tty"

    print(prompt, end="", flush=True)
    waited = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            print()
            if line == "":
                return "eof"
            if line.strip().lower() in ("q", "quit", "exit"):
                return "quit"
            return "next"
        waited += 0.1
        if reminder_every > 0 and waited >= reminder_every:
            waited = 0.0
            print("  (still waiting - press Enter in this terminal to continue, 'q' to quit)")

    return "shutdown"


class TreeLocationVisualiser(Node):
    def __init__(self, args, ground_truth: List[GroundTruthTree]) -> None:
        super().__init__("tree_location_visualiser")
        self.args = args
        self.ground_truth = ground_truth

        self.detect_client = self.create_client(DetectYolo, "/yolo/detect")
        self.process_client = self.create_client(
            ProcessTreeLocations, "/yolo/process_tree_locations"
        )

        # Plain name (not private) so the RViz config can point straight at it.
        self.marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, LATCHED_QOS)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cycle = 0

        # Optional Gazebo markers (translucent poles in the sim world).
        self.marker_files = write_marker_templates() if args.gazebo else {}
        self._last_gazebo_sig = None
        if args.gazebo:
            self._clear_gazebo_markers()

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

    # -------------------------------------------------------------- markers
    @staticmethod
    def _base_marker(ns: str, marker_id: int, action: int = Marker.ADD) -> Marker:
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = marker_id
        m.action = action
        return m

    def _delete_all_markers(self) -> List[Marker]:
        return [
            self._base_marker(ns, 0, Marker.DELETEALL)
            for ns in (NS_TREES, NS_LABELS, NS_GT)
        ]

    def _build_markers(self, locations) -> MarkerArray:
        array = MarkerArray()
        array.markers.extend(self._delete_all_markers())

        now = self.get_clock().now().to_msg()

        for i, loc in enumerate(locations):
            r, g, b = TYPE_COLORS.get(loc.tree_type, UNKNOWN_COLOR)

            sphere = self._base_marker(NS_TREES, i)
            sphere.header.stamp = now
            sphere.type = Marker.SPHERE
            sphere.pose.position.x = loc.location.x
            sphere.pose.position.y = loc.location.y
            sphere.pose.position.z = loc.location.z + 0.75
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.5
            sphere.color.r, sphere.color.g, sphere.color.b = r, g, b
            sphere.color.a = 0.95
            array.markers.append(sphere)

            label = self._base_marker(NS_LABELS, i)
            label.header.stamp = now
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = loc.location.x
            label.pose.position.y = loc.location.y
            label.pose.position.z = loc.location.z + 2.4
            label.text = f"{loc.tree_type}\n({loc.location.x:.1f}, {loc.location.y:.1f})"
            label.scale.z = 0.9
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            array.markers.append(label)

        if self.args.show_ground_truth:
            # Every ground-truth tree in the world, as a plain grey marker.
            for i, t in enumerate(self.ground_truth):
                gt = self._base_marker(NS_GT, i)
                gt.header.stamp = now
                gt.type = Marker.SPHERE
                gt.pose.position.x = t.x
                gt.pose.position.y = t.y
                gt.pose.position.z = t.z + 0.6
                gt.pose.orientation.w = 1.0
                gt.scale.x = gt.scale.y = gt.scale.z = 1.2
                gt.color.r, gt.color.g, gt.color.b = GROUND_TRUTH_COLOR
                gt.color.a = 0.5
                array.markers.append(gt)

        return array

    # ---------------------------------------------------------------- cycle
    def _nearest_gt_error(self, loc) -> Optional[float]:
        if not self.ground_truth:
            return None
        return min(
            math.hypot(t.x - loc.location.x, t.y - loc.location.y)
            for t in self.ground_truth
        )

    def run_cycle(self) -> int:
        self.cycle += 1

        req = DetectYolo.Request()
        req.start_yolo = True
        detections = self._call(self.detect_client, req).detections

        preq = ProcessTreeLocations.Request()
        preq.start_lidar = True
        preq.detections = detections
        locations = self._call(self.process_client, preq).tree_data

        self.marker_pub.publish(self._build_markers(locations))
        if self.args.gazebo:
            self._update_gazebo_markers(locations)

        types = {}
        errors = []
        for loc in locations:
            types[loc.tree_type] = types.get(loc.tree_type, 0) + 1
            err = self._nearest_gt_error(loc)
            if err is not None:
                errors.append(err)

        summary = ", ".join(f"{k}:{v}" for k, v in sorted(types.items())) or "none"
        err_txt = f", nearest-GT error max {max(errors):.2f} m" if errors else ""
        self.get_logger().info(
            f"[cycle {self.cycle}] {len(detections.bounding_boxes)} boxes -> "
            f"{len(locations)} trees ({summary}){err_txt} -> published to {MARKER_TOPIC}"
        )
        return len(locations)

    # --------------------------------------------------------------- gazebo
    def _clear_gazebo_markers(self) -> None:
        """Remove any marker poles left over from a previous cycle/run."""
        for i in range(GAZEBO_MARKER_MAX):
            gazebo_remove_model(self.args.world, f"tree_marker_{i}")

    def _update_gazebo_markers(self, locations) -> None:
        """Place a translucent coloured pole in Gazebo at each detected tree."""
        signature = tuple(
            (loc.tree_type, round(loc.location.x, 2), round(loc.location.y, 2))
            for loc in locations
        )
        if signature == self._last_gazebo_sig:
            return  # nothing changed; avoid needless spawn/remove churn
        self._last_gazebo_sig = signature

        self._clear_gazebo_markers()
        spawned = 0
        for i, loc in enumerate(locations[:GAZEBO_MARKER_MAX]):
            key = loc.tree_type if loc.tree_type in self.marker_files else "UNKNOWN"
            z = loc.location.z + GAZEBO_MARKER_LENGTH / 2.0
            if gazebo_spawn_model(
                self.args.world, f"tree_marker_{i}", self.marker_files[key],
                loc.location.x, loc.location.y, z,
            ):
                spawned += 1
        self.get_logger().info(f"Gazebo: placed {spawned} marker pole(s)")

    def wait_for_base_pose(self, target_xy, tolerance: float = 1.5, timeout: float = 15.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    "world", "parrot1_base_link", Time(), timeout=Duration(seconds=0.5)
                )
                dx = tf.transform.translation.x - target_xy[0]
                dy = tf.transform.translation.y - target_xy[1]
                if math.hypot(dx, dy) <= tolerance:
                    return True
            except Exception:  # noqa: BLE001
                pass
        return False

    def spin_for(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> int:
    ap = argparse.ArgumentParser(description="Visualise detected tree locations as RViz markers.")
    ap.add_argument("--tour", action=argparse.BooleanOptionalAction, default=False,
                    help="Teleport the drone around the forest, one tree per stop.")
    ap.add_argument("--loop", action=argparse.BooleanOptionalAction, default=False,
                    help="With --tour, keep repeating the tour instead of stopping at the end.")
    ap.add_argument("--views", type=int, default=6, help="Number of stops in a tour.")
    ap.add_argument("--view-distance", type=float, default=15.0, help="Stand-off distance from the tree (m).")
    ap.add_argument("--camera-height", type=float, default=4.0, help="Height above the tree base (m).")
    ap.add_argument("--min-separation", type=float, default=40.0, help="Minimum spacing between stops (m).")
    ap.add_argument("--settle-time", type=float, default=3.0, help="Seconds to wait after a teleport (s).")
    ap.add_argument("--interval", type=float, default=5.0, help="Seconds between refreshes (s).")
    ap.add_argument("--step", action=argparse.BooleanOptionalAction, default=False,
                    help="Wait for you to press Enter before moving on: the next tour stop, "
                         "or (without --tour) the next detection run at the current pose. "
                         "Type 'q' + Enter to stop. Needs an interactive terminal, so run "
                         "this with `ros2 run` rather than from a launch file.")
    ap.add_argument("--world", default="v2_forest")
    ap.add_argument("--world-file", default="")
    ap.add_argument("--model-name", default="parrot1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--show-ground-truth", action=argparse.BooleanOptionalAction, default=False,
                    help="Also draw a grey marker at every true tree position (from the SDF).")
    ap.add_argument("--gazebo", action=argparse.BooleanOptionalAction, default=False,
                    help="Also spawn coloured marker poles in the Gazebo world "
                         "(/world/<world>/create) at each detected tree, so the result "
                         "can be checked against the actual trees in the sim view.")
    ap.add_argument("--rviz", action=argparse.BooleanOptionalAction, default=False,
                    help="Also launch RViz with config/tree_locations.rviz.")
    # When started as a ROS node (e.g. from a launch file), launch appends
    # `--ros-args -r __node:=...`; drop those before argparse sees them.
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    random.seed(args.seed)

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

    # Launch RViz with the bundled config before we start spamming the log.
    rviz_proc = None
    if args.rviz:
        config = Path(get_package_share_directory("rs1_perception")) / "config" / "tree_locations.rviz"
        print(f"Launching RViz with {config}")
        rviz_proc = subprocess.Popen(["rviz2", "-d", str(config)])

    rclpy.init()
    node = TreeLocationVisualiser(args, ground_truth)

    try:
        if args.tour:
            viewpoints = choose_viewpoints(
                ground_truth, args.views, args.min_separation
            )
            if not viewpoints:
                print("No ground-truth trees to build a tour from; use without --tour")
                return 2

            print(f"\nStarting tour of {len(viewpoints)} viewpoints. "
                  f"Markers on {MARKER_TOPIC} (fixed frame 'world')."
                  + (" Press Enter at each stop to continue." if args.step else ""))
            quitting = False
            while rclpy.ok() and not quitting:
                for i, target in enumerate(viewpoints):
                    if target.x >= 0:
                        dx, dy, yaw = target.x - args.view_distance, target.y, 0.0
                    else:
                        dx, dy, yaw = target.x + args.view_distance, target.y, math.pi
                    dz = target.z + args.camera_height
                    dx = max(-95.0, min(95.0, dx))
                    dy = max(-95.0, min(95.0, dy))

                    print(f"\n[{i + 1}/{len(viewpoints)}] -> ({dx:.1f}, {dy:.1f}, {dz:.1f}), "
                          f"target {target.name} ({target.tree_type})")
                    if set_pose(args.model_name, args.world, dx, dy, dz, yaw):
                        node.wait_for_base_pose((dx, dy))
                    else:
                        print("  set_pose failed; using the current pose")
                    node.spin_for(args.settle_time)

                    node.run_cycle()

                    if args.step:
                        answer = wait_for_user_input(
                            node, "  press Enter for the next stop, or 'q' to quit > "
                        )
                        if answer in ("quit", "eof", "shutdown"):
                            quitting = True
                            break
                    elif i < len(viewpoints) - 1:
                        node.spin_for(args.interval)

                if quitting or not args.loop:
                    break
                print("\nTour complete - looping")
        else:
            pace = ("each time you press Enter" if args.step
                    else f"every {args.interval:.0f}s")
            print(f"\nRefreshing at the drone's current pose {pace}. "
                  f"Markers on {MARKER_TOPIC} (fixed frame 'world'). Ctrl+C to stop.")
            while rclpy.ok():
                node.run_cycle()
                if args.step:
                    answer = wait_for_user_input(
                        node, "  press Enter to re-run detection here, or 'q' to quit > "
                    )
                    if answer in ("quit", "eof", "shutdown"):
                        break
                else:
                    node.spin_for(args.interval)
    except (KeyboardInterrupt, RuntimeError) as ex:
        if isinstance(ex, RuntimeError):
            print(f"ERROR: {ex}")
        return 0 if isinstance(ex, KeyboardInterrupt) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if rviz_proc is not None:
            rviz_proc.terminate()

    print("\nDone. Markers remain latched on the topic until the node restarts.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
