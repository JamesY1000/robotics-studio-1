#!/usr/bin/env python3
"""Turns YOLO tree detections into world-frame tree base locations.

1. For each BoundingBox sample the RGBD depth image.
2. Transform that point to the world frame using TF (global for now this may need to change.).
3. Confirm with the Parrot lidar (help with 80% YOLO detction accuracy).
"""

import math
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener

from rs1_interfaces.msg import TreeLocation
from rs1_interfaces.srv import ProcessTreeLocations

_DEPTH_32FC1 = "32FC1"
_DEPTH_16UC1 = ("16UC1", "mono16")


def _quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix (3x3) from a quaternion."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _angdiff(a: float, b: float) -> float:
    """Smallest signed difference between two angles."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class TreeLocationNode(Node):
    def __init__(self):
        super().__init__("tree_location_node")

        # Topics / frames
        self.declare_parameter("depth_image_topic", "/parrot1/camera/depth/image")
        self.declare_parameter("lidar_topic", "/parrot1/scan")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_link_frame", "parrot1_base_link")
        self.declare_parameter("lidar_frame", "parrot1_base_scan")
        self.declare_parameter("depth_optical_frame", "parrot1_camera_depth_optical_frame")

        # Camera intrinsics (fallback when derived from resolution + hfov)
        self.declare_parameter("hfov", 2.0944)

        # Depth sampling
        self.declare_parameter("min_depth", 0.4)
        self.declare_parameter("max_depth", 40.0)
        self.declare_parameter("depth_patch", 5)
        self.declare_parameter("min_box_pixels", 4)

        # False-positive filtering: a detection box must actually show foliage
        # (trees render dark grey; sky/terrain/background do not). Mirrors the
        # heuristic validated in sample_generator's camera_collector.
        self.declare_parameter("rgb_image_topic", "/parrot1/camera/image")
        self.declare_parameter("dark_threshold", 100.0)
        # Real YOLO tree boxes are >50% dark foliage, so 0.25 drops boxes on
        # sky/terrain/background without discarding genuine trees.
        self.declare_parameter("min_dark_fraction", 0.25)
        self.declare_parameter("require_foliage_check", True)

        # Lidar confirmation
        self.declare_parameter("lidar_angle_tolerance", 0.03)  # rad (~1.7 deg)
        self.declare_parameter("lidar_range_min", 0.2)
        self.declare_parameter("lidar_range_max", 40.0)
        # Maximum allowed difference between the depth-derived horizontal range
        # and the lidar range for the tree to be considered confirmed.
        self.declare_parameter("max_range_difference", 3.0)
        self.declare_parameter("require_lidar_confirmation", True)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.depth_optical_frame = self.get_parameter("depth_optical_frame").value
        self.hfov = float(self.get_parameter("hfov").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.depth_patch = int(self.get_parameter("depth_patch").value)
        self.min_box_pixels = int(self.get_parameter("min_box_pixels").value)
        self.rgb_image_topic = self.get_parameter("rgb_image_topic").value
        self.dark_threshold = float(self.get_parameter("dark_threshold").value)
        self.min_dark_fraction = float(self.get_parameter("min_dark_fraction").value)
        self.require_foliage_check = bool(self.get_parameter("require_foliage_check").value)
        self.lidar_angle_tolerance = float(self.get_parameter("lidar_angle_tolerance").value)
        self.lidar_range_min = float(self.get_parameter("lidar_range_min").value)
        self.lidar_range_max = float(self.get_parameter("lidar_range_max").value)
        self.max_range_difference = float(self.get_parameter("max_range_difference").value)
        self.require_lidar_confirmation = bool(self.get_parameter("require_lidar_confirmation").value)

        self.depth_msg: Optional[Image] = None
        self.rgb_msg: Optional[Image] = None
        self.scan_msg: Optional[LaserScan] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image,
            self.get_parameter("depth_image_topic").value,
            self._depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.rgb_image_topic,
            self._rgb_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("lidar_topic").value,
            self._scan_cb,
            qos_profile_sensor_data,
        )

        self.process_srv = self.create_service(
            ProcessTreeLocations, "/yolo/process_tree_locations", self.process_callback
        )

        self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info("tree_location_node ready")

    def _on_param_update(self, params):
        """Allow the tuning thresholds to be changed at runtime."""
        for p in params:
            if p.name == "min_dark_fraction":
                self.min_dark_fraction = float(p.value)
            elif p.name == "dark_threshold":
                self.dark_threshold = float(p.value)
            elif p.name == "max_range_difference":
                self.max_range_difference = float(p.value)
            elif p.name == "lidar_angle_tolerance":
                self.lidar_angle_tolerance = float(p.value)
            elif p.name == "require_foliage_check":
                self.require_foliage_check = bool(p.value)
            elif p.name == "require_lidar_confirmation":
                self.require_lidar_confirmation = bool(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------- callbacks
    def _depth_cb(self, msg: Image) -> None:
        self.depth_msg = msg

    def _rgb_cb(self, msg: Image) -> None:
        self.rgb_msg = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan_msg = msg

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _decode_depth(msg: Image) -> Optional[np.ndarray]:
        """Depth image in metres, or None if the encoding is unsupported."""
        try:
            if msg.encoding == _DEPTH_32FC1:
                return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
            if msg.encoding in _DEPTH_16UC1:
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
                return arr.astype(np.float32) / 1000.0
            return None
        except Exception as ex:  # noqa: BLE001
            return None

    @staticmethod
    def _decode_bgr(msg: Image) -> Optional[np.ndarray]:
        """Decode a sensor_msgs/Image into a BGR uint8 array."""
        try:
            if msg.encoding == "bgr8":
                return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return arr[:, :, ::-1]
            if msg.encoding in ("mono8", "8UC1"):
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)
                return np.repeat(arr[:, :, None], 3, axis=2)
        except Exception:  # noqa: BLE001
            return None
        return None

    def _box_shows_foliage(self, left: int, top: int, right: int, bottom: int) -> Optional[bool]:
        """Helps with false positives - looks at foliage to determine if tree is there or if it si just skybox
        """
        if self.rgb_msg is None:
            return None
        img = self._decode_bgr(self.rgb_msg)
        if img is None:
            return None
        h, w = img.shape[:2]
        y0, y1 = max(0, top), min(h, bottom + 1)
        x0, x1 = max(0, left), min(w, right + 1)
        if y1 <= y0 or x1 <= x0:
            return False
        gray = img[y0:y1, x0:x1].astype(np.float32).mean(axis=2)
        dark_fraction = float((gray < self.dark_threshold).mean())
        return dark_fraction >= self.min_dark_fraction

    def _lookup(self, parent: str, child: str):
        """Latest transform parent <- child, or None."""
        try:
            return self.tf_buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"Could not transform {child} -> {parent}: {ex}")
            return None

    def _depth_point_camera_frame(self, depth: np.ndarray, u: int, v: int) -> Optional[np.ndarray]:
        """3D point (optical convention) in the camera frame at pixel (u, v)."""
        h, w = depth.shape[:2]
        r = self.depth_patch
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32).reshape(-1)
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.min_depth) & (valid <= self.max_depth)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))

        fx = (w / 2.0) / math.tan(self.hfov / 2.0)
        fy = fx
        cx, cy = w / 2.0, h / 2.0
        x = (u - cx) / fx * z
        y = (v - cy) / fy * z
        return np.array([x, y, z])  # optical: x right, y down, z forward

    def _lidar_ranges_by_world_azimuth(self):
        scan = self.scan_msg
        if scan is None:
            return None, None, None

        trafo = self._lookup(self.world_frame, self.lidar_frame)
        if trafo is None:
            return None, None, None

        t = trafo.transform.translation
        q = trafo.transform.rotation
        R = _quaternion_to_matrix(q.x, q.y, q.z, q.w)
        origin = np.array([t.x, t.y, t.z])

        ranges_arr = np.asarray(scan.ranges, dtype=np.float64)
        n = ranges_arr.size
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        valid = np.isfinite(ranges_arr)
        valid &= ranges_arr >= max(self.lidar_range_min, scan.range_min)
        valid &= ranges_arr <= min(self.lidar_range_max, scan.range_max)
        if not np.any(valid):
            return origin[:2], np.empty(0), np.empty(0)

        angles = angles[valid]
        ranges_arr = ranges_arr[valid]

        # Beam endpoints in the lidar frame, rotated into the world frame.
        pts_lidar = np.stack(
            [ranges_arr * np.cos(angles), ranges_arr * np.sin(angles), np.zeros_like(ranges_arr)],
            axis=1,
        )
        pts_world = pts_lidar @ R.T + origin

        dx = pts_world[:, 0] - origin[0]
        dy = pts_world[:, 1] - origin[1]
        azimuths = np.arctan2(dy, dx)
        horizontal = np.hypot(dx, dy)
        return origin[:2], azimuths, horizontal

    def _lidar_range_at(self, origin_xy, azimuths, ranges, az: float) -> Optional[float]:
        if azimuths is None or azimuths.size == 0:
            return None
        mask = np.abs(np.array([_angdiff(a, az) for a in azimuths])) <= self.lidar_angle_tolerance
        if not np.any(mask):
            return None
        # The nearest echo in the tree's azimuth is its front surface; a median
        # would be pulled outwards by background beams in the same sector.
        return float(np.min(ranges[mask]))

    def process_callback(self, request, response):
        response.tree_data = []

        if not request.start_lidar:
            self.get_logger().warn("start_lidar=false; nothing to process")
            return response

        detections = request.detections
        if detections is None or len(detections.bounding_boxes) == 0:
            self.get_logger().warn("No detections supplied; nothing to process")
            return response

        if self.depth_msg is None:
            self.get_logger().warn("No depth image received yet; returning no tree locations")
            return response

        depth = self._decode_depth(self.depth_msg)
        if depth is None:
            self.get_logger().warn(f"Unsupported depth encoding '{self.depth_msg.encoding}'")
            return response

        # Camera frame used for the depth points (prefer the message header, so
        # the correct optical/link convention is applied).
        header_frame = (self.depth_msg.header.frame_id or "").strip()
        cam_frame = header_frame if header_frame else self.depth_optical_frame
        is_optical = "optical" in cam_frame
        trafo_cam = self._lookup(self.world_frame, cam_frame)
        if trafo_cam is None:
            return response

        t = trafo_cam.transform.translation
        q = trafo_cam.transform.rotation
        R_wc = _quaternion_to_matrix(q.x, q.y, q.z, q.w)
        cam_origin = np.array([t.x, t.y, t.z])

        lidar_origin, scan_az, scan_ranges = self._lidar_ranges_by_world_azimuth()
        have_lidar = lidar_origin is not None and scan_az is not None and scan_az.size > 0
        if self.require_lidar_confirmation and not have_lidar:
            self.get_logger().warn("Lidar confirmation required but no usable scan; returning no locations")
            return response

        h, w = depth.shape[:2]
        img_h, img_w = self.depth_msg.height, self.depth_msg.width

        kept = 0
        dropped_no_tree = 0
        for bb in detections.bounding_boxes:
            left = max(0, min(bb.left, bb.right))
            right = min(img_w - 1, max(bb.left, bb.right))
            top = max(0, min(bb.top, bb.bottom))
            bottom = min(img_h - 1, max(bb.top, bb.bottom))
            if (right - left) < self.min_box_pixels or (bottom - top) < self.min_box_pixels:
                continue

            # Drop false positives: a bounding box that does not actually show a
            # tree (no foliage in the box region).
            if self.require_foliage_check:
                shows = self._box_shows_foliage(left, top, right, bottom)
                if shows is False:
                    dropped_no_tree += 1
                    continue

            # Sample candidate pixels near the bottom of the box (where the tree
            # meets the ground) and keep the one whose depth best agrees with the
            # lidar range measured in the same azimuth.
            candidates = []
            n_cols = 5
            for row_frac in (1.0, 0.85, 0.7):
                v = max(0, min(int(round(top + row_frac * (bottom - top))), h - 1))
                for i in range(n_cols):
                    u = max(0, min(int(round(left + (i + 0.5) / n_cols * (right - left))), w - 1))
                    candidates.append((u, v))

            # Pick the candidate pixel whose depth best agrees with the lidar
            # range in the same azimuth (most self-consistent sample), then place
            # the tree using the authoritative lidar range and the depth bearing.
            best = None       # (score, base_x, base_y, base_z)
            fallback = None   # (base_x, base_y, base_z) from the first valid sample
            for (u, v) in candidates:
                pt_opt = self._depth_point_camera_frame(depth, u, v)
                if pt_opt is None:
                    continue
                # Optical (x right, y down, z forward) -> camera_link
                # (x fwd, y left, z up) when the point is in a link frame.
                pt_cam = pt_opt if is_optical else np.array([pt_opt[2], -pt_opt[0], -pt_opt[1]])
                pt_world = R_wc @ pt_cam + cam_origin

                dx = pt_world[0] - lidar_origin[0]
                dy = pt_world[1] - lidar_origin[1]
                r_depth = math.hypot(dx, dy)
                if r_depth < 1e-6:
                    continue
                az = math.atan2(dy, dx)
                if fallback is None:
                    fallback = (pt_world[0], pt_world[1], pt_world[2])

                if not have_lidar:
                    if best is None or r_depth < best[0]:
                        best = (r_depth, pt_world[0], pt_world[1], pt_world[2])
                    continue

                r_lidar = self._lidar_range_at(lidar_origin, scan_az, scan_ranges, az)
                if r_lidar is None:
                    continue
                score = abs(r_lidar - r_depth)
                if best is None or score < best[0]:
                    base_x = lidar_origin[0] + r_lidar * math.cos(az)
                    base_y = lidar_origin[1] + r_lidar * math.sin(az)
                    best = (score, base_x, base_y, pt_world[2])

            if best is None:
                if self.require_lidar_confirmation or fallback is None:
                    continue
                base_x, base_y, base_z = fallback
            else:
                score, base_x, base_y, base_z = best
                if have_lidar and score > self.max_range_difference:
                    if self.require_lidar_confirmation:
                        continue
                    if fallback is None:
                        continue
                    base_x, base_y, base_z = fallback

            tree = TreeLocation()
            tree.tree_type = bb.class_name
            tree.location = Point(x=float(base_x), y=float(base_y), z=float(base_z))
            response.tree_data.append(tree)
            kept += 1

        self.get_logger().info(
            f"Processed {len(detections.bounding_boxes)} detection(s) -> {kept} tree location(s) "
            f"({dropped_no_tree} dropped: no tree in box)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TreeLocationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
