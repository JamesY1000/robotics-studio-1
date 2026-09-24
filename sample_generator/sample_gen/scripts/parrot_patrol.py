#!/usr/bin/env python3
"""Teleports the Parrot around the rs1_ws forest and collects one YOLO sample
per viewpoint using the camera_collector's ``collect_sample`` service.

This replaces ``sample_generator.py`` for the labelled-forest workflow: the
trees are already present in the world (each carrying the ObjectLabelPlugin),
so this node only moves the drone and asks the collector to capture the current
frame. Waypoints are either read from ``patrol_config.yml`` or derived
automatically from the tree TFs published on ``/tf_object_models``.
"""

import math
import random
import time
from dataclasses import dataclass
from typing import Any, Dict, List

import rclpy
from rclpy.client import Client
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from sample_gen.base import BaseNode, BaseParams
from sample_gen.services import (
    SERVICE_COLLECT_SAMPLE,
    SERVICE_GET_CURRENT_SAMPLE_ID,
    SERVICE_UPDATE_GT_SUBS,
)
from sample_gen_msgs.srv import CollectSample, GetCurrentSampleID
from sample_gen.helpers import (
    create_ros_service_client,
    get_pose_msg,
    load_yaml_entry,
    set_pose_process,
)

NODE_NAME = "parrot_patrol_node"
SAMPLE_GEN_PKG = "sample_gen"
TREE_FRAME_PREFIX = "generated_tree_"
TF_OBJECT_MODELS_TOPIC = "/tf_object_models"


@dataclass
class ParrotPatrolParams(BaseParams):
    world_name: str = "v2_forest"
    parrot_model_name: str = "parrot1"
    # When true, viewpoints are derived from the tree TFs published by the
    # ObjectLabelPlugin (one random viewpoint per tree). When false, waypoints
    # are loaded from waypoints_yml / waypoints_key.
    derive_from_trees: bool = True
    num_viewpoints: int = 0            # max viewpoints to visit (0 = all trees)
    view_distance: float = 15.0        # horizontal distance camera -> tree (m)
    camera_height: float = 3.0         # camera altitude above the tree base (m)
    min_altitude: float = 2.0          # absolute floor for the viewpoint z
    min_clearance: float = 4.0         # min dist from viewpoint to any other tree base (m)
    max_viewpoint_tries: int = 30      # random angles tried per tree before skipping
    min_trees: int = 50                # trees to wait for before starting
    wait_for_trees_timeout: float = 120.0  # s to wait for tree TFs to appear
    waypoints_yml: str = "patrol_config.yml"
    waypoints_key: str = "parrot_waypoints"
    wait_after_teleport: float = 0.5
    max_collect_retries: int = 500     # retries per viewpoint (0.2 s each)
    num_passes: int = 1                # how many full passes over the waypoints to make
    target_samples: int = 0            # stop once this many samples exist (0 = ignore)
    seed: int = 0                      # random seed (0 = none)


class ParrotPatrol(BaseNode):
    def __init__(self):
        self._params: ParrotPatrolParams = ParrotPatrolParams()
        super().__init__(NODE_NAME)

        self.collect_sample_client: Client = create_ros_service_client(
            SERVICE_COLLECT_SAMPLE, CollectSample, self
        )
        self.update_gt_subs_client: Client = create_ros_service_client(
            SERVICE_UPDATE_GT_SUBS, Trigger, self
        )
        self.get_current_sample_id_client: Client = create_ros_service_client(
            SERVICE_GET_CURRENT_SAMPLE_ID, GetCurrentSampleID, self
        )

        # Tree world poses (child_frame_id -> latest transform) published by the
        # ObjectLabelPlugin instances on the global /tf_object_models topic.
        self._tree_tfs: Dict[str, Any] = {}
        self._tree_sub = self.create_subscription(
            TFMessage, TF_OBJECT_MODELS_TOPIC, self._tree_tf_cb, 100
        )

        self.get_logger().info("Initialized")

    # ------------------------------------------------------------------ trees
    def _tree_tf_cb(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.child_frame_id.startswith(TREE_FRAME_PREFIX):
                self._tree_tfs[tf.child_frame_id] = tf

    def _wait_for_trees(self) -> bool:
        deadline = time.monotonic() + self._params.wait_for_trees_timeout
        last_new_tree = time.monotonic()
        while time.monotonic() < deadline:
            before = len(self._tree_tfs)
            # spin_once so the /tf_object_models subscription is serviced.
            rclpy.spin_once(self, timeout_sec=1.0)
            if len(self._tree_tfs) > before:
                last_new_tree = time.monotonic()
                self.get_logger().info(f"Discovered {len(self._tree_tfs)} tree frames")
            elif (len(self._tree_tfs) >= self._params.min_trees
                  and time.monotonic() - last_new_tree >= 5.0):
                # Count has stopped growing -> the world finished loading.
                self.get_logger().info(f"Tree discovery stable at {len(self._tree_tfs)} trees")
                return True
        self.get_logger().warn(
            f"Tree discovery did not stabilise (have {len(self._tree_tfs)}/{self._params.min_trees})"
        )
        return len(self._tree_tfs) >= self._params.min_trees

    def _derive_waypoints(self) -> List[List[float]]:
        if not self._wait_for_trees():
            self.get_logger().error(
                f"Timed out waiting for tree TFs (have {len(self._tree_tfs)}), aborting"
            )
            return []

        names = list(self._tree_tfs.keys())
        if self._params.seed:
            random.seed(self._params.seed)
        random.shuffle(names)
        count = self._params.num_viewpoints or len(names)
        names = names[:count]

        # Tree positions (x, y) used to keep the drone out of other canopies.
        positions = [
            (t.transform.translation.x, t.transform.translation.y, n)
            for n, t in self._tree_tfs.items()
        ]

        # The forest bounds (from the tree positions) so the drone never flies
        # out past the perimeter walls, where it would film the wall backsides
        # instead of the trees.
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        area_min_x, area_max_x = min(xs), max(xs)
        area_min_y, area_max_y = min(ys), max(ys)

        waypoints: List[List[float]] = []
        for name in names:
            tf = self._tree_tfs[name]
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z

            # Pick a viewpoint at view_distance around the target tree that is
            # inside the forest bounds, and far enough from every OTHER tree so
            # the drone does not end up inside a canopy with its camera blocked.
            chosen = None
            for _ in range(self._params.max_viewpoint_tries):
                angle = random.uniform(-math.pi, math.pi)
                radius = self._params.view_distance
                vx = tx + radius * math.cos(angle)
                vy = ty + radius * math.sin(angle)
                if not (area_min_x <= vx <= area_max_x and area_min_y <= vy <= area_max_y):
                    continue
                if not self._too_close_to_other_trees(vx, vy, name, positions):
                    chosen = (vx, vy)
                    break
            if chosen is None:
                self.get_logger().warn(f"No clear viewpoint found for {name}, skipping it")
                continue

            vx, vy = chosen
            vz = max(tz + self._params.camera_height, self._params.min_altitude)
            yaw = math.atan2(ty - vy, tx - vx)
            waypoints.append([vx, vy, vz, yaw])
        return waypoints

    def _too_close_to_other_trees(self, vx: float, vy: float, target_name: str, positions: list) -> bool:
        min_dist_sq = self._params.min_clearance ** 2
        for px, py, name in positions:
            if name == target_name:
                continue
            dx = vx - px
            dy = vy - py
            if dx * dx + dy * dy < min_dist_sq:
                return True
        return False

    def _load_waypoints(self) -> List[List[float]]:
        if self._params.derive_from_trees:
            return self._derive_waypoints()
        raw = load_yaml_entry(
            self._params.waypoints_key, SAMPLE_GEN_PKG, self._params.waypoints_yml
        )
        return [list(map(float, wp)) for wp in raw]

    # ------------------------------------------------------------- patrol loop
    def _call_service(self, client: Client, request: Any) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _teleport(self, waypoint: List[float]) -> bool:
        pose = get_pose_msg(x=waypoint[0], y=waypoint[1], z=waypoint[2], yaw=waypoint[3])
        if not set_pose_process(self, pose, self._params.world_name, self._params.parrot_model_name):
            self.get_logger().error(f"Failed to teleport parrot to {waypoint}")
            return False
        time.sleep(self._params.wait_after_teleport)
        return True

    def _collect_sample(self) -> bool:
        request = CollectSample.Request()
        # Stamp the request ONCE with the moment the drone finished moving. The
        # collector rejects frames/TFs older than this + a freshness margin, so
        # keep the stamp fixed across retries (advancing it would re-stale data).
        request.stamp = self.get_clock().now().to_msg()
        attempts = 0
        while True:
            response = self._call_service(self.collect_sample_client, request)
            if response.success:
                return True
            if 0 < self._params.max_collect_retries <= attempts:
                self.get_logger().error(
                    f"Collect sample failed after {attempts} retries"
                )
                return False
            attempts += 1
            time.sleep(0.2)

    def run(self) -> None:
        # Wait until the labelled trees are loaded (their TFs appear on
        # /tf_object_models) BEFORE re-scanning ObjectLabel topics, otherwise
        # the collector subscribes to nothing and no boxes are produced.
        if not self._wait_for_trees():
            self.get_logger().error("No tree TFs appeared, shutting down")
            return
        # Give ROS DDS discovery a moment, then re-scan ObjectLabel topics so
        # the collector subscribes to every labelled tree. Scan twice with a
        # delay because some plugins advertise their label topic a little after
        # their TF.
        time.sleep(1.0)
        self._call_service(self.update_gt_subs_client, Trigger.Request())
        time.sleep(2.0)
        self._call_service(self.update_gt_subs_client, Trigger.Request())
        time.sleep(1.0)

        for pass_idx in range(self._params.num_passes):
            waypoints = self._load_waypoints()
            if not waypoints:
                self.get_logger().error("No waypoints available, shutting down")
                return

            self.get_logger().info(
                f"Pass {pass_idx + 1}/{self._params.num_passes}: {len(waypoints)} waypoints"
            )
            for i, wp in enumerate(waypoints):
                if self._target_reached():
                    self.get_logger().info("Target sample count reached, stopping")
                    return
                self.get_logger().info(f"Waypoint {i + 1}/{len(waypoints)}: {wp}")
                if not self._teleport(wp):
                    continue
                self._collect_sample()

        self.get_logger().info("Patrol complete")

    def _target_reached(self) -> bool:
        if self._params.target_samples <= 0:
            return False
        resp = self._call_service(self.get_current_sample_id_client, GetCurrentSampleID.Request())
        return resp.sample_id >= self._params.target_samples


def main(args=None):
    rclpy.init(args=args)
    node = ParrotPatrol()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
