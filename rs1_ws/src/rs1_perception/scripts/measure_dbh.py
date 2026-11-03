#!/usr/bin/env python3
"""Measure the requested tree using fresh LiDAR scans while Husky is parked."""

import math
import json
from collections import deque
from pathlib import Path
from statistics import median
from threading import Condition, Lock
from time import monotonic, time_ns
from typing import NamedTuple

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.serialization import serialize_message
from rs1_interfaces.srv import LocateTree, MeasureDBH
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from dbh_3d_detector import DetectionParams, pointcloud_to_numpy, trunk_widths
from level_husky_cloud import rotation


class TrunkFit(NamedTuple):
    diameter: float
    offset_x: float
    offset_y: float


class DBHService(Node):
    def __init__(self):
        super().__init__("dbh_service")
        self.params = DetectionParams()
        self.params.lidar_height = self.declare_parameter("lidar_height", -1.0).value
        self.scan_count = self.declare_parameter("scan_count", 5).value
        self.timeout = self.declare_parameter("measurement_timeout_s", 7.0).value
        self.min_range = self.declare_parameter("tree_measurement_min_m", 1.0).value
        self.max_range = self.declare_parameter("tree_measurement_max_m", 4.0).value
        self.center_tolerance = self.declare_parameter("target_tolerance_m", 0.35).value
        self.max_spread = self.declare_parameter("max_dbh_spread_m", 0.04).value
        self.center_spread = self.declare_parameter("max_center_spread_m", 0.12).value
        self.diagnostics_dir = self.declare_parameter("diagnostics_dir", "").value
        self.diagnostic_requests = 0
        self.max_diagnostic_requests = self.declare_parameter("max_diagnostic_requests", 64).value
        self.capture = None
        self.scan_diagnostics = None
        limits = (self.timeout, self.center_tolerance, self.max_spread, self.center_spread)
        if self.scan_count < 2 or any(not math.isfinite(v) or v <= 0 for v in limits):
            raise ValueError("DBH needs at least two scans and positive tolerances and timeout")
        if (not math.isfinite(self.min_range) or not math.isfinite(self.max_range)
                or not 0 < self.min_range < self.max_range <= 4.0):
            raise ValueError("Invalid DBH measurement range")

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.cloud_ready = Condition()
        self.latest = None
        self.measuring = Lock()
        self.create_subscription(PointCloud2, "/husky1/scan/points", self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_service(MeasureDBH, "/husky1/measure_dbh", self.measure,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(LocateTree, "/husky1/locate_tree", self.locate,
                            callback_group=ReentrantCallbackGroup())

    def on_cloud(self, cloud):
        # Only run the detector when Husky actually asks for a measurement
        with self.cloud_ready:
            self.latest = cloud
            self.cloud_ready.notify()

    def measure(self, request, response):
        limit = request.max_center_offset_m
        if not math.isfinite(limit) or limit < 0:
            response.message = "Invalid target tolerance"
            return response
        tolerance = min(limit, self.center_tolerance) if limit else self.center_tolerance
        fits, reason = self.sample(request.map_header.frame_id, request.tree.location,
                                   tolerance, False)
        response.success = bool(fits)
        if fits:
            response.dbh_m = float(median(f.diameter for f in fits))
            response.message = f"Measured tree {request.tree_index} from {len(fits)} fresh scans"
        else:
            response.message = f"Tree {request.tree_index}: {reason}"
        self.get_logger().info(response.message)
        return response

    def locate(self, request, response):
        # A large map error is only recoverable when one standing trunk fits the search area
        fits, response.message = self.sample(request.map_header.frame_id, request.expected,
                                             request.search_radius_m, True)
        response.success = bool(fits)
        if fits:
            response.centre.x = request.expected.x + median(f.offset_x for f in fits)
            response.centre.y = request.expected.y + median(f.offset_y for f in fits)
            response.centre.z = request.expected.z
        return response

    def sample(self, frame, tree, tolerance, locating):
        if (not frame or not 0 < tolerance <= 2
                or not all(math.isfinite(v) for v in (tree.x, tree.y, tree.z))):
            return [], "Missing frame or invalid tree search"
        if not self.measuring.acquire(blocking=False):
            return [], "Husky is already measuring a tree"
        try:
            self.capture = deque(maxlen=8) if self.diagnostics_dir and self.diagnostic_requests < self.max_diagnostic_requests else None
            result = self.collect(frame, tree, tolerance, locating)
            if self.capture is not None:
                self.save_diagnostics(frame, tree, tolerance, locating, result)
            return result
        finally:
            self.capture = None
            self.scan_diagnostics = None
            self.measuring.release()

    def save_diagnostics(self, frame, tree, tolerance, locating, result):
        self.diagnostic_requests += 1
        folder = Path(self.diagnostics_dir) / f"{time_ns()}_{'locate' if locating else 'dbh'}"
        try:
            folder.mkdir(parents=True)
            metadata = dict(frame=frame, target=[tree.x, tree.y, tree.z], tolerance=tolerance,
                            locating=locating, params=vars(self.params), success=bool(result[0]),
                            message=result[1], fits=[list(f) for f in result[0]],
                            measurement_range=[self.min_range, self.max_range])
            (folder / "request.json").write_text(json.dumps(metadata, indent=2))
            for index, (cloud, details) in enumerate(self.capture):
                # Keep the original message too so replay uses exactly the scan that was rejected
                np.savez_compressed(folder / f"scan_{index:02d}.npz",
                                    cloud_cdr=np.frombuffer(serialize_message(cloud), dtype=np.uint8),
                                    xyz=pointcloud_to_numpy(cloud),
                                    details=np.array(json.dumps(details)))
            self.get_logger().info(f"Tree perception capture: {folder}: {result[1]}")
        except OSError as error:
            self.get_logger().warning(f"Could not save tree perception capture: {error}")

    def collect(self, frame, tree, tolerance, locating):
        # Use ROS time for scan stamps and wall time so a paused sim still times out
        last_stamp = self.get_clock().now().nanoseconds
        deadline = monotonic() + self.timeout
        count = min(3, self.scan_count) if locating else self.scan_count
        readings = deque(maxlen=count)
        reason = "No fresh scans"
        previous = None
        while rclpy.ok() and monotonic() < deadline:
            with self.cloud_ready:
                remaining = deadline - monotonic()
                if not self.cloud_ready.wait_for(lambda: self.latest is not previous, remaining):
                    break
                cloud = self.latest
                previous = cloud
            stamp = Time.from_msg(cloud.header.stamp)
            if not last_stamp < stamp.nanoseconds <= self.get_clock().now().nanoseconds:
                continue
            last_stamp = stamp.nanoseconds
            # Keep the rejection reason even when saving raw scans is switched off
            self.scan_diagnostics = dict(cloud_frame=cloud.header.frame_id, stamp_ns=stamp.nanoseconds)
            try:
                fit, reason = self.match_scan(frame, tree, tolerance, locating, cloud, stamp)
            except (TransformException, ValueError, KeyError, AssertionError,
                    np.linalg.LinAlgError) as error:
                fit, reason = None, str(error)
            if self.capture is not None:
                self.scan_diagnostics.update(reason=reason, fit=list(fit) if fit else None)
                self.capture.append((cloud, self.scan_diagnostics))
            if fit is None:
                readings.clear()
                continue
            readings.append(fit)
            if len(readings) < count:
                continue

            # Similar diameters alone can't tell two neighbouring trunks apart
            if math.hypot(np.ptp([r.offset_x for r in readings]),
                          np.ptp([r.offset_y for r in readings])) > self.center_spread:
                reason = "Fitted trunk centre moved between scans"
                continue
            diameters = [r.diameter for r in readings]
            if not locating and max(diameters) - min(diameters) > self.max_spread:
                reason = "Diameter varied too much between scans"
                continue
            if monotonic() >= deadline:
                break
            return readings, f"Confirmed trunk from {count} fresh scans"
        return [], f"{reason} ({len(readings)}/{count} scans)"

    def match_scan(self, frame, tree, tolerance, locating, cloud, stamp):
        transform = self.tf.lookup_transform(
            cloud.header.frame_id, frame, stamp, timeout=Duration(seconds=0.2)).transform
        t = transform.translation
        rot = rotation(transform.rotation)
        target = rot @ [tree.x, tree.y, tree.z + self.params.breast_height] + [t.x, t.y, t.z]
        if self.scan_diagnostics is not None:
            self.scan_diagnostics.update(target_scanner=target.tolist(),
                                         rotation=rot.tolist(), translation=[t.x, t.y, t.z])
        distance = np.linalg.norm(target[:2])
        if locating and distance-tolerance > self.params.max_range:
            return None, (f"Mapped target is {distance:.2f} m from LiDAR; even with "
                          f"{tolerance:.2f} m map uncertainty it is beyond the {self.params.max_range:.2f} m locating limit")
        if not locating and not self.min_range <= distance <= self.max_range:
            return None, self.range_reason("Target", distance)

        stems = trunk_widths(pointcloud_to_numpy(cloud), self.params, target[:2], tolerance,
                             locating=locating, diagnostics=self.scan_diagnostics)
        if stems is None:
            return None, "Could not establish ground height"
        if len(stems) != 1:
            if stems:
                return None, "Ambiguous trunk match: multiple trunks in the search area; move to a clearer view"
            tracks = (self.scan_diagnostics or {}).get("tracks", [])
            if tracks and not locating:
                heights = [height for track in tracks for height in track['heights']]
                if max(heights) < self.params.breast_height-self.params.height_tolerance:
                    return None, (f"No matching trunk at breast height: visible stem reaches only {max(heights):.2f} m "
                                  f"above ground, need a clear view near {self.params.breast_height:.2f} m; move around the tree")
                reasons = sorted({track['reason'] for track in tracks if track.get('reason')})
                if reasons:
                    return None, "No matching trunk: " + "; ".join(reasons) + "; move to a clearer view"
            return None, "No matching trunk: insufficient clear stem returns near the selected tree; move closer or around it"
        diameter, centre = stems[0]
        fitted_distance = np.linalg.norm(centre)
        if locating and fitted_distance > self.params.max_range:
            return None, f"Located trunk is {fitted_distance:.2f} m from LiDAR, beyond the {self.params.max_range:.2f} m locating limit"
        if not locating and not self.min_range <= fitted_distance <= self.max_range:
            return None, self.range_reason("Fitted trunk", fitted_distance)
        # Compare the centres in the request frame even when the scanner tilts between scans
        offset = rot.T @ [*(centre - target[:2]), 0.0]
        return TrunkFit(diameter, offset[0], offset[1]), "Collecting fresh scans"

    def range_reason(self, label, distance):
        change = (f"move at least {distance-self.max_range:.2f} m closer" if distance > self.max_range
                  else f"move at least {self.min_range-distance:.2f} m farther away")
        return (f"{label} is outside the measurement stand-off: {distance:.2f} m from LiDAR, "
                f"allowed {self.min_range:.2f}–{self.max_range:.2f} m; {change}")


def main():
    rclpy.init()
    node = DBHService()
    # The service waits for new scans while the cloud and TF callbacks keep running
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
