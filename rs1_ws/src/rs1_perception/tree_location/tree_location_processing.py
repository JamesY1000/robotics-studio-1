#!/usr/bin/env python3
"""Turns YOLO tree detections into world-frame tree base locations.

1. For each BoundingBox sample the RGBD depth image.
2. Transform that point to the world frame using TF (global for now this may need to change.).
3. Confirm with the Parrot lidar: the FULL 3D scan cloud (all 9 vertical rings)
   is used to check that the detection sits on a vertical trunk column, which
   works at any flight altitude.  The single level ring is only a fallback.
"""

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, LaserScan, PointCloud2
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
        self.declare_parameter("use_truth_odometry", False)
        self.declare_parameter("odometry_topic", "/parrot1/odometry")

        # Camera intrinsics (fallback when derived from resolution + hfov)
        self.declare_parameter("hfov", 2.0944)

        # Depth sampling
        self.declare_parameter("min_depth", 0.4)
        self.declare_parameter("max_depth", 40.0)
        self.declare_parameter("depth_patch", 5)
        self.declare_parameter("min_box_pixels", 4)
        # Restrict detection sampling to the lower trunk region; the canopy can
        # drift the centroid away from the true tree base in oak crowns.
        self.declare_parameter("trunk_lower_fraction", 0.35)

        # False-positive filtering: a detection box must actually show foliage
        # (trees render dark grey; sky/terrain/background do not). Mirrors the
        # heuristic validated in sample_generator's camera_collector.
        self.declare_parameter("rgb_image_topic", "/parrot1/camera/image")
        self.declare_parameter("dark_threshold", 100.0)
        # Real YOLO tree boxes are >50% dark foliage, so 0.25 drops boxes on
        # sky/terrain/background without discarding genuine trees.
        self.declare_parameter("min_dark_fraction", 0.25)
        self.declare_parameter("require_foliage_check", True)

        # Trunk check: drop boxes whose lower band has no near vertical trunk
        # column (e.g. the tree is off-screen and only canopy is visible, which
        # biases the location toward the canopy instead of the trunk base).
        self.declare_parameter("require_trunk_check", True)
        self.declare_parameter("trunk_depth_gap", 0.5)
        self.declare_parameter("trunk_min_width_frac", 0.05)
        self.declare_parameter("trunk_max_width_frac", 0.6)

        # Lidar confirmation
        self.declare_parameter("lidar_angle_tolerance", 0.03)  # rad (~1.7 deg)
        self.declare_parameter("lidar_range_min", 0.2)
        self.declare_parameter("lidar_range_max", 40.0)
        # Maximum allowed difference between the depth-derived horizontal range
        # and the lidar range for the tree to be considered confirmed.
        self.declare_parameter("max_range_difference", 3.0)
        self.declare_parameter("require_lidar_confirmation", True)

        # Lidar trunk echo cluster: a trunk shows up as a short contiguous run
        # of beams whose range is near the front surface.  The angular width of
        # that run -> trunk/cone width, used for the base position and (if
        # enabled) species verification.
        self.declare_parameter("lidar_cluster_depth_gap", 0.3)   # m; beams closer than front+gap belong to the trunk
        self.declare_parameter("lidar_cluster_min_beams", 2)     # minimum contiguous beams for a valid trunk
        self.declare_parameter("lidar_cluster_max_width_m", 3.0) # wider than this is canopy/background, not a trunk

        # Full 3D scan cloud (all vertical rings).  The 360x9 lidar is bridged
        # twice: the ring is published as a LaserScan and the whole cloud as a
        # PointCloud2.  The bridge picks the MIDDLE ring for the LaserScan, so the
        # ring is always level with the drone.  That is fine while the drone flies
        # at trunk height, but the moment the altitude changes (duck to 0.8 m, or a
        # pop-up to 8-12 m to clear the canopy) the level ring sails over or under
        # the trunk and every detection is dropped as "no lidar trunk echo".
        # The cloud lets the trunk be recognised as a VERTICAL COLUMN at any
        # altitude, because the 9 rays sweep +/-15 deg and always cross it.
        self.declare_parameter("pointcloud_topic", "/parrot1/scan/points")
        self.declare_parameter("use_pointcloud", True)
        # Transforming a stale cloud with the current pose would place it where
        # the drone used to be, so ignore clouds older than this.
        self.declare_parameter("cloud_max_age_s", 2.0)
        self.declare_parameter("cloud_angle_tolerance", 0.09)  # rad (~5 deg); trunk-search sector
        self.declare_parameter("cloud_depth_gap", 0.6)         # m; points within front +/- gap belong to the trunk
        self.declare_parameter("cloud_min_points", 6)         # minimum points in the trunk cluster
        self.declare_parameter("cloud_level_bin_m", 0.5)       # m; z bin size for the column test
        self.declare_parameter("cloud_min_levels", 2)         # z levels needed before the column test applies
        self.declare_parameter("cloud_column_max_spread_m", 0.5)  # max lateral drift of the per-level centres
        self.declare_parameter("cloud_min_column_span_m", 1.0)    # a trunk spans height; a branch sheet does not
        self.declare_parameter("cloud_width_percentile", 85.0)    # robust lateral-width percentile

        # ONE lidar subscription.  The gpu_lidar fans 25 rings over +/-45 deg, so
        # the horizontal centre ring is at elevation 0 and is exactly what
        # /parrot1/scan publishes.  Carving that slice back out of the cloud means
        # the ring test, the trunk-column test and the bush guard all read the
        # SAME set of points, and the node no longer subscribes to the LaserScan
        # as well.  Set false to fall back to the /parrot1/scan subscription (a
        # node restart is required - the subscription is created at startup).
        self.declare_parameter("ring_from_cloud", True)
        self.declare_parameter("ring_bins", 360)                # azimuth bins; matches the 1 deg ring
        self.declare_parameter("ring_elevation_tolerance", 0.0327)  # rad; half the 3.75 deg ring spacing

        # Per-frame dedup: two locations this close (m) within one response are
        # the same tree seen twice -> keep the highest-confidence one.
        self.declare_parameter("frame_dedup_distance", 1.0)

        # Verbose per-detection diagnostics (bearing, ranges, drop reasons).
        self.declare_parameter("debug", False)

        # Lidar trunk validation for OAKS: a visible oak trunk measures NARROW at
        # the lidar height.  A wide echo means branches/foliage only (trunk not
        # showing), so the detection is dropped rather than placing a centroid on
        # the branches.  Pines are cones and are intentionally allowed wide.
        # Species itself comes from YOLO + the merger's per-type vote, NOT lidar.
        self.declare_parameter("oak_trunk_max_width_m", 0.8)

        # BUSH GUARD (needs the full cloud).  Bushes are the only thing in this
        # world that YOLO will call a tree but that is not one: the model has no
        # bush class and no negative examples, so a bush is boxed as an OAK.
        # Measured geometry (rs1_sim/models, do not re-derive):
        #   bush_2 collision cylinder r=1.25, len 1.5 -> 2.5 m wide, tops at 1.5 m
        #   bush_3 collision cylinder r=1.35, len 2.1 -> 2.7 m wide, tops at 2.1 m
        #   oak trunk 0.1-0.8 m wide; pine cone base ~1.1-1.5 m
        # So a bush is both far TOO WIDE to be a trunk and a LOW blob with only
        # distant surfaces above it, whereas a trunk continues up into its canopy.
        self.declare_parameter("trunk_max_width_m", 1.8)          # species-independent width ceiling
        self.declare_parameter("bush_sector_rad", 0.35)          # rad (~20 deg); WIDE, so a whole bush face fits
        self.declare_parameter("bush_lateral_gap_m", 0.3)         # m; lateral break between separate objects
        self.declare_parameter("canopy_probe_depth_m", 1.2)       # m; gather the whole object face, not one arc
        self.declare_parameter("canopy_gap_m", 0.5)               # m; vertical gap above the object before looking up
        self.declare_parameter("trunk_canopy_max_gap_m", 2.5)     # m; trunk much nearer than canopy -> bush

        self.world_frame = self.get_parameter("world_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.depth_optical_frame = self.get_parameter("depth_optical_frame").value
        self.use_truth_odometry = bool(self.get_parameter("use_truth_odometry").value)
        self.odometry_topic = self.get_parameter("odometry_topic").value
        self.hfov = float(self.get_parameter("hfov").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.depth_patch = int(self.get_parameter("depth_patch").value)
        self.min_box_pixels = int(self.get_parameter("min_box_pixels").value)
        self.trunk_lower_fraction = float(self.get_parameter("trunk_lower_fraction").value)
        self.rgb_image_topic = self.get_parameter("rgb_image_topic").value
        self.dark_threshold = float(self.get_parameter("dark_threshold").value)
        self.min_dark_fraction = float(self.get_parameter("min_dark_fraction").value)
        self.require_foliage_check = bool(self.get_parameter("require_foliage_check").value)
        self.require_trunk_check = bool(self.get_parameter("require_trunk_check").value)
        self.trunk_depth_gap = float(self.get_parameter("trunk_depth_gap").value)
        self.trunk_min_width_frac = float(self.get_parameter("trunk_min_width_frac").value)
        self.trunk_max_width_frac = float(self.get_parameter("trunk_max_width_frac").value)
        self.lidar_angle_tolerance = float(self.get_parameter("lidar_angle_tolerance").value)
        self.lidar_range_min = float(self.get_parameter("lidar_range_min").value)
        self.lidar_range_max = float(self.get_parameter("lidar_range_max").value)
        self.max_range_difference = float(self.get_parameter("max_range_difference").value)
        self.require_lidar_confirmation = bool(self.get_parameter("require_lidar_confirmation").value)
        self.lidar_cluster_depth_gap = float(self.get_parameter("lidar_cluster_depth_gap").value)
        self.lidar_cluster_min_beams = int(self.get_parameter("lidar_cluster_min_beams").value)
        self.lidar_cluster_max_width_m = float(self.get_parameter("lidar_cluster_max_width_m").value)
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.use_pointcloud = bool(self.get_parameter("use_pointcloud").value)
        self.ring_from_cloud = bool(self.get_parameter("ring_from_cloud").value)
        self.ring_bins = int(self.get_parameter("ring_bins").value)
        self.ring_elevation_tolerance = float(
            self.get_parameter("ring_elevation_tolerance").value
        )
        self.cloud_max_age_s = float(self.get_parameter("cloud_max_age_s").value)
        self.cloud_angle_tolerance = float(self.get_parameter("cloud_angle_tolerance").value)
        self.cloud_depth_gap = float(self.get_parameter("cloud_depth_gap").value)
        self.cloud_min_points = int(self.get_parameter("cloud_min_points").value)
        self.cloud_level_bin_m = float(self.get_parameter("cloud_level_bin_m").value)
        self.cloud_min_levels = int(self.get_parameter("cloud_min_levels").value)
        self.cloud_column_max_spread_m = float(self.get_parameter("cloud_column_max_spread_m").value)
        self.cloud_min_column_span_m = float(self.get_parameter("cloud_min_column_span_m").value)
        self.cloud_width_percentile = float(self.get_parameter("cloud_width_percentile").value)
        self.frame_dedup_distance = float(self.get_parameter("frame_dedup_distance").value)
        self.oak_trunk_max_width_m = float(self.get_parameter("oak_trunk_max_width_m").value)
        self.trunk_max_width_m = float(self.get_parameter("trunk_max_width_m").value)
        self.bush_sector_rad = float(self.get_parameter("bush_sector_rad").value)
        self.bush_lateral_gap_m = float(self.get_parameter("bush_lateral_gap_m").value)
        self.canopy_probe_depth_m = float(self.get_parameter("canopy_probe_depth_m").value)
        self.canopy_gap_m = float(self.get_parameter("canopy_gap_m").value)
        self.trunk_canopy_max_gap_m = float(self.get_parameter("trunk_canopy_max_gap_m").value)
        self.debug = bool(self.get_parameter("debug").value)

        self.depth_msg: Optional[Image] = None
        self.rgb_msg: Optional[Image] = None
        self.scan_msg: Optional[LaserScan] = None
        self.cloud_msg: Optional[PointCloud2] = None
        self.odometry_msg = None

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
        # The LaserScan subscription is only needed when the level ring is NOT
        # derived from the cloud (see `ring_from_cloud`).
        if not self.ring_from_cloud:
            self.create_subscription(
                LaserScan,
                self.get_parameter("lidar_topic").value,
                self._scan_cb,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._cloud_cb,
            qos_profile_sensor_data,
        )
        if self.use_truth_odometry:
            self.create_subscription(
                Odometry, self.odometry_topic, self._odometry_cb, qos_profile_sensor_data
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
            elif p.name == "require_trunk_check":
                self.require_trunk_check = bool(p.value)
            elif p.name == "trunk_depth_gap":
                self.trunk_depth_gap = float(p.value)
            elif p.name == "trunk_min_width_frac":
                self.trunk_min_width_frac = float(p.value)
            elif p.name == "trunk_max_width_frac":
                self.trunk_max_width_frac = float(p.value)
            elif p.name == "require_lidar_confirmation":
                self.require_lidar_confirmation = bool(p.value)
            elif p.name == "lidar_cluster_depth_gap":
                self.lidar_cluster_depth_gap = float(p.value)
            elif p.name == "lidar_cluster_min_beams":
                self.lidar_cluster_min_beams = int(p.value)
            elif p.name == "lidar_cluster_max_width_m":
                self.lidar_cluster_max_width_m = float(p.value)
            elif p.name == "frame_dedup_distance":
                self.frame_dedup_distance = float(p.value)
            elif p.name == "debug":
                self.debug = bool(p.value)
            elif p.name == "oak_trunk_max_width_m":
                self.oak_trunk_max_width_m = float(p.value)
            elif p.name == "trunk_max_width_m":
                self.trunk_max_width_m = float(p.value)
            elif p.name == "bush_sector_rad":
                self.bush_sector_rad = float(p.value)
            elif p.name == "bush_lateral_gap_m":
                self.bush_lateral_gap_m = float(p.value)
            elif p.name == "canopy_probe_depth_m":
                self.canopy_probe_depth_m = float(p.value)
            elif p.name == "canopy_gap_m":
                self.canopy_gap_m = float(p.value)
            elif p.name == "trunk_canopy_max_gap_m":
                self.trunk_canopy_max_gap_m = float(p.value)
            elif p.name == "use_pointcloud":
                self.use_pointcloud = bool(p.value)
            elif p.name == "cloud_angle_tolerance":
                self.cloud_angle_tolerance = float(p.value)
            elif p.name == "cloud_depth_gap":
                self.cloud_depth_gap = float(p.value)
            elif p.name == "cloud_min_points":
                self.cloud_min_points = int(p.value)
            elif p.name == "cloud_level_bin_m":
                self.cloud_level_bin_m = float(p.value)
            elif p.name == "cloud_min_levels":
                self.cloud_min_levels = int(p.value)
            elif p.name == "cloud_column_max_spread_m":
                self.cloud_column_max_spread_m = float(p.value)
            elif p.name == "cloud_min_column_span_m":
                self.cloud_min_column_span_m = float(p.value)
            elif p.name == "cloud_width_percentile":
                self.cloud_width_percentile = float(p.value)
            elif p.name == "cloud_max_age_s":
                self.cloud_max_age_s = float(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------- callbacks
    def _depth_cb(self, msg: Image) -> None:
        self.depth_msg = msg

    def _rgb_cb(self, msg: Image) -> None:
        self.rgb_msg = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan_msg = msg

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self.cloud_msg = msg

    def _odometry_cb(self, msg) -> None:
        self.odometry_msg = msg

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

    def _find_trunk_column(self, depth: np.ndarray, left: int, top: int, right: int, bottom: int):
        """Locate the trunk column in the lower band of the box.

        A trunk shows up as a narrow, near-vertical, contiguous run of image
        columns whose nearest depth is clearly closer than the canopy/background
        around it.  Returns ``(u_center, v_base, depth_median, width_px)`` in
        pixel coordinates, or ``None`` when no plausible trunk is visible (e.g.
        only canopy is in frame, or the drone is above the canopy).
        """
        height = bottom - top + 1
        band_top = max(top, bottom - int(round(self.trunk_lower_fraction * height)))
        if band_top >= bottom:
            return None
        band = depth[band_top:bottom + 1, left:right + 1].astype(np.float32)
        valid = np.isfinite(band) & (band >= self.min_depth) & (band <= self.max_depth)
        if not np.any(valid):
            return None

        # Nearest depth in each image column within the lower band.
        col_min = np.where(valid, band, np.inf).min(axis=0)
        finite_cols = np.isfinite(col_min)
        if not np.any(finite_cols):
            return None

        # Reference: the canopy/background behind the trunk (70th percentile, so
        # a narrow trunk does not drag the reference toward itself).
        ref = float(np.percentile(col_min[finite_cols], 70.0))
        near_cols = finite_cols & (col_min < (ref - self.trunk_depth_gap))
        if not np.any(near_cols):
            return None

        # The near columns must form one contiguous run of plausible trunk width.
        box_w = right - left + 1
        min_run = max(2, int(round(self.trunk_min_width_frac * box_w)))
        max_run = max(min_run, int(round(self.trunk_max_width_frac * box_w)))
        best = (0, -1, -1)
        run = 0
        start = 0
        for i in range(near_cols.size):
            if near_cols[i]:
                if run == 0:
                    start = i
                run += 1
            else:
                if run > best[0]:
                    best = (run, start, i - 1)
                run = 0
        if run > best[0]:
            best = (run, start, int(near_cols.size) - 1)
        run_len, run_start, run_end = best
        if run_len < min_run or run_len > max_run:
            return None

        u_center = left + (run_start + run_end) // 2
        width_px = run_end - run_start + 1

        # Representative depth of the trunk front from the run's columns.
        col_slice = band[:, run_start:run_end + 1]
        col_valid = np.isfinite(col_slice) & (col_slice >= self.min_depth) & (col_slice <= self.max_depth)
        if not np.any(col_valid):
            return None
        depth_median = float(np.median(col_slice[col_valid]))

        # Lowest row in the band where the trunk run still has a valid return
        # (the visible trunk base).  Falls back to the box bottom.
        rows_with_valid = np.where(col_valid.any(axis=1))[0]
        v_base = bottom if rows_with_valid.size == 0 else band_top + int(rows_with_valid[-1])

        return u_center, v_base, depth_median, width_px

    def _box_has_trunk(self, depth: np.ndarray, left: int, top: int, right: int, bottom: int) -> bool:
        """True when the lower band of the box contains a near vertical trunk."""
        return self._find_trunk_column(depth, left, top, right, bottom) is not None

    def _trunk_is_credible(self, class_name: str, width_m: Optional[float]) -> bool:
        """True if the lidar echo is consistent with a visible TRUNK.

        An oak's visible trunk measures narrow at the lidar height; a wide echo
        means we are seeing branches/foliage only, so a centroid placed there
        would land on the branches -> reject.  Pines are cones and are
        intentionally allowed to be wide.  Species is decided by YOLO + the
        merger's per-type vote, not here.
        """
        if not self.require_trunk_check or width_m is None:
            return True
        if class_name == "OAK_TREE":
            return width_m <= self.oak_trunk_max_width_m
        return True

    def _lookup(self, parent: str, child: str):
        """Latest transform parent <- child, or None."""
        try:
            return self.tf_buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"Could not transform {child} -> {parent}: {ex}")
            return None

    @staticmethod
    def _compose(p1, q1, p2, q2):
        """Compose world_T_base (p1,q1) with base_T_sensor (p2,q2) -> world_T_sensor."""
        R1 = _quaternion_to_matrix(q1.x, q1.y, q1.z, q1.w)
        p = np.array([p1.x, p1.y, p1.z]) + R1 @ np.array([p2.x, p2.y, p2.z])
        # quaternion multiply q1 * q2
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        return p, (x / n, y / n, z / n, w / n)

    def _sensor_in_world(self, sensor_frame: str):
        """(origin_xyz, R_world_sensor) for a sensor frame.

        Use the registered localisation TF chain by default. The optional truth
        mode composes Gazebo's base pose with the static sensor mounting transform.
        Return None if the selected pose source is unavailable.
        """
        if not self.use_truth_odometry:
            trafo = self._lookup(self.world_frame, sensor_frame)
            if trafo is None:
                return None
            t = trafo.transform.translation
            q = trafo.transform.rotation
            R = _quaternion_to_matrix(q.x, q.y, q.z, q.w)
            return np.array([t.x, t.y, t.z]), R

        odom = self.odometry_msg
        if odom is None:
            return None
        base = self._lookup(self.base_link_frame, sensor_frame)
        if base is None:
            return None
        origin, quat = self._compose(
            odom.pose.pose.position, odom.pose.pose.orientation,
            base.transform.translation, base.transform.rotation,
        )
        R = _quaternion_to_matrix(quat[0], quat[1], quat[2], quat[3])
        return origin, R

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

    def _pixel_bearing_world(self, cam_frame: str, u: int, v: int):
        """World azimuth of the ray through pixel (u, v), independent of depth.

        Returns ``(az_world, cam_origin_xyz)`` or ``None``.  Used to seed the
        lidar search on the canopy centroid (directly above an upright trunk)
        without trusting the depth image, which is biased toward near branches
        in a dense forest.
        """
        h, w = self.depth_msg.height, self.depth_msg.width
        fx = (w / 2.0) / math.tan(self.hfov / 2.0)
        fy = fx
        cx, cy = w / 2.0, h / 2.0
        d_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d_opt /= np.linalg.norm(d_opt)
        # optical (x right, y down, z fwd) -> link (x fwd, y left, z up)
        d_link = np.array([d_opt[2], -d_opt[0], -d_opt[1]])
        sensor = self._sensor_in_world(cam_frame)
        if sensor is None:
            return None
        cam_origin, R_wc = sensor
        d_world = R_wc @ d_link
        az = float(math.atan2(d_world[1], d_world[0]))
        return az, cam_origin

    def _trunk_box_samples(self, left: int, top: int, right: int, bottom: int):
        """Deprecated helper kept for compatibility; use ``_find_trunk_column``."""
        if right <= left or bottom <= top:
            return []
        return []

    def _lidar_scan_data(self, cloud=None):
        """Latest lidar ring in a cluster-friendly form.

        Returns ``(origin_xy, beam_azimuths_world, beam_ranges, angle_increment)``
        or ``None`` when no usable scan/TF is available.  ``beam_ranges`` keeps
        the original per-beam ordering (invalid beams are left as-is) so a
        contiguous trunk echo can be found by beam index.

        With ``ring_from_cloud`` (the default) the ring is carved out of the
        already-decoded 3D cloud, so the ring test and the trunk-column / bush
        tests all read the SAME points and only one lidar topic is subscribed.
        """
        if self.ring_from_cloud:
            if cloud is not None:
                ring = self._ring_from_cloud(cloud[2], cloud[0], cloud[1])
                if ring is not None:
                    return cloud[1], ring[0], ring[1], ring[2]
            return None

        scan = self.scan_msg
        if scan is None:
            return None

        sensor = self._sensor_in_world(self.lidar_frame)
        if sensor is None:
            return None
        origin, R = sensor

        ranges_arr = np.asarray(scan.ranges, dtype=np.float64)
        n = ranges_arr.size
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        # Unit beam directions in the lidar frame, rotated into the world frame.
        dirs_lidar = np.stack(
            [np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1
        )
        dirs_world = dirs_lidar @ R.T
        azimuths = np.arctan2(dirs_world[:, 1], dirs_world[:, 0])
        return origin[:2], azimuths, ranges_arr, scan.angle_increment

    def _lidar_cluster_at(self, origin_xy, azimuths, ranges, angle_increment, az: float, target_range=None):
        """Contiguous trunk echo around the tree's bearing.

        Seeds on the valid beam within ``lidar_angle_tolerance`` of ``az`` whose
        range is CLOSEST to ``target_range`` (the tree's expected horizontal
        range from the canopy-centroid depth pixel), so a nearer neighbour trunk
        or branch cannot steal the seed in a dense forest.  Falls back to the
        nearest beam when ``target_range`` is unavailable.  Grows a contiguous
        (in beam-index) run while each neighbour's range stays within
        ``lidar_cluster_depth_gap`` of the front surface.
        Returns ``(mean_range, az_center, width_m, n_beams)`` or ``None``.
        """
        if azimuths is None or azimuths.size == 0:
            return None

        finite = np.isfinite(ranges)
        finite &= ranges >= max(self.lidar_range_min, 0.0)
        finite &= ranges <= self.lidar_range_max

        diffs = np.abs(np.array([_angdiff(a, az) for a in azimuths]))
        cand = finite & (diffs <= self.lidar_angle_tolerance)
        if not np.any(cand):
            return None

        cand_idx = np.flatnonzero(cand)
        if target_range is not None and math.isfinite(target_range) and target_range > 0.0:
            seed = int(cand_idx[np.argmin(np.abs(ranges[cand_idx] - target_range))])
        else:
            seed = int(cand_idx[np.argmin(ranges[cand_idx])])
        seed_range = float(ranges[seed])

        # Grow left/right over contiguous valid beams at the trunk front.
        lo = seed
        for i in range(seed - 1, -1, -1):
            if finite[i] and ranges[i] <= seed_range + self.lidar_cluster_depth_gap:
                lo = i
            else:
                break
        hi = seed
        for i in range(seed + 1, int(ranges.size)):
            if finite[i] and ranges[i] <= seed_range + self.lidar_cluster_depth_gap:
                hi = i
            else:
                break

        n_beams = hi - lo + 1
        if n_beams < self.lidar_cluster_min_beams:
            return None

        cluster = ranges[lo:hi + 1]
        mean_range = float(np.median(cluster))
        # Trunk-centre bearing: circular mean of the cluster's beam azimuths, so
        # the tree x,y is anchored on the LIDAR's own measurement of the trunk
        # direction (more accurate than the depth-pixel bearing, which drifts
        # with the optical/link unprojection).
        azs = azimuths[lo:hi + 1]
        az_center = float(np.arctan2(np.mean(np.sin(azs)), np.mean(np.cos(azs))))
        width_rad = (n_beams - 1) * angle_increment
        width_m = 2.0 * mean_range * math.sin(width_rad / 2.0)
        if width_m > self.lidar_cluster_max_width_m:
            # Wider than a trunk/cone at the lidar height: canopy or background.
            return None
        return mean_range, az_center, width_m, n_beams

    # ------------------------------------------------ full point-cloud trunk
    @staticmethod
    def _decode_pointcloud(msg: PointCloud2) -> Optional[np.ndarray]:
        """``(N, 3)`` float32 xyz array from a PointCloud2, or ``None``.

        Decoded here rather than via ``sensor_msgs_py`` so the node does not gain
        a dependency and so any field layout works: only the x/y/z FLOAT32
        offsets are read and everything else in a point (intensity, ring, ...) is
        skipped using ``point_step``.
        """
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")):
            return None
        if any(int(fields[k].datatype) != 7 for k in ("x", "y", "z")):  # 7 = FLOAT32
            return None
        step = int(msg.point_step)
        if step <= 0:
            return None
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        n = min(int(msg.width) * max(1, int(msg.height)), raw.size // step)
        if n <= 0:
            return None
        rows = raw[: n * step].reshape(n, step)
        pts = np.empty((n, 3), dtype=np.float32)
        for i, key in enumerate(("x", "y", "z")):
            off = int(fields[key].offset)
            if off < 0 or off + 4 > step:
                return None
            pts[:, i] = rows[:, off:off + 4].copy().view(np.float32).ravel()
        return pts

    def _cloud_points_world(self):
        """Lidar cloud transformed into the world frame.

        Returns ``(pts_world, origin_xy, pts_link, origin_z)``.  ``pts_link`` is
        the raw cloud in the SENSOR frame and is what the level-ring slice is
        taken from (the ring tilts with the drone, so a world-z test would pick a
        different ring every time the drone pitches).

        Returns ``None`` when the cloud has not arrived, is stale, cannot be
        decoded, or the sensor pose is not available.
        """
        msg = self.cloud_msg
        if not self.use_pointcloud or msg is None:
            return None

        # The cloud is placed using the CURRENT pose, so an old cloud would be
        # rendered where the drone used to be (at 3 m/s that matters a lot).
        try:
            age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
            if 0.0 <= age and age > self.cloud_max_age_s:
                return None
        except Exception:  # noqa: BLE001
            pass

        pts = self._decode_pointcloud(msg)
        if pts is None or pts.size == 0:
            return None
        frame = (msg.header.frame_id or "").strip() or self.lidar_frame
        sensor = self._sensor_in_world(frame)
        if sensor is None:
            return None
        origin, R = sensor

        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            return None
        pts_link = pts.astype(np.float64)
        pts_world = pts_link @ R.T + origin
        return pts_world, origin[:2], pts_link, float(origin[2])

    def _ring_from_cloud(self, pts_link, pts_world, origin_xy):
        """Level ring carved out of the 3D cloud: ``(azimuths, ranges, increment)``.

        The horizontal centre ring of the gpu_lidar is at elevation 0, so slicing
        the sensor-frame cloud to ``|z| <= tan(tol) * r`` rebuilds exactly the
        scan a 1 deg ring publishes.  Each azimuth bin keeps its NEAREST return,
        so a trunk front wins over anything behind it - the same beam behaviour
        ``_lidar_cluster_at`` was written against.  Empty bins stay NaN, which
        terminates a contiguous cluster run exactly like an invalid beam does.
        """
        n = int(self.ring_bins)
        if n < 36 or pts_link is None or pts_world is None or pts_world.size == 0:
            return None

        r_link = np.hypot(pts_link[:, 0], pts_link[:, 1])
        band = (r_link > 0.05) & (
            np.abs(pts_link[:, 2]) <= math.tan(self.ring_elevation_tolerance) * r_link
        )
        if not np.any(band):
            return None

        dx = pts_world[band, 0] - origin_xy[0]
        dy = pts_world[band, 1] - origin_xy[1]
        ranges = np.hypot(dx, dy)
        az = np.arctan2(dy, dx)

        increment = 2.0 * math.pi / float(n)
        bins = np.floor((az + math.pi) / increment).astype(np.int64) % n
        order = np.lexsort((ranges, bins))
        bins_sorted = bins[order]
        uniq, first = np.unique(bins_sorted, return_index=True)

        ring = np.full(n, np.nan)
        ring[uniq] = ranges[order][first]
        # Bin centres, so a trunk centred on a ring sample still maps to one bin.
        azimuths = -math.pi + (np.arange(n) + 0.5) * increment
        return azimuths, ring, increment

    def _cloud_trunk_at(self, pts_world, origin_xy, az_seed: float, target_range=None):
        """Find the trunk column in the FULL lidar cloud at a given bearing.

        Unlike the level ring, the 9 vertical rays of the cloud sweep +/-15 deg, so
        the trunk is crossed whatever altitude the drone is at (duck at 0.8 m,
        cruise at 2.2 m, pop-up at 8-12 m).  The trunk is identified as a
        *vertical column*: the same lateral centre at every height.  Branches and
        canopy give a different centre per level, which is exactly what the level
        ring could not tell apart when the drone was off trunk height.

        Returns ``(range_m, az_centre, width_m, n_levels, column_spread_m)`` or
        ``None`` when no plausible trunk column is in the sector.
        """
        if pts_world is None or len(pts_world) == 0:
            return None

        dx = pts_world[:, 0] - origin_xy[0]
        dy = pts_world[:, 1] - origin_xy[1]
        rng = np.hypot(dx, dy)
        use = np.isfinite(rng) & (rng >= self.lidar_range_min) & (rng <= self.lidar_range_max)
        if not np.any(use):
            return None

        az = np.arctan2(dy, dx)
        diff = np.abs((az - az_seed + math.pi) % (2.0 * math.pi) - math.pi)
        cand = use & (diff <= self.cloud_angle_tolerance)
        if not np.any(cand):
            return None

        # Seed on the echo closest to the expected range (from the canopy-centroid
        # depth pixel) so a nearer neighbour trunk cannot steal the seed.
        cand_idx = np.flatnonzero(cand)
        if target_range is not None and math.isfinite(target_range) and target_range > 0.0:
            seed = int(cand_idx[np.argmin(np.abs(rng[cand_idx] - target_range))])
        else:
            seed = int(cand_idx[np.argmin(rng[cand_idx])])
        front = float(rng[seed])

        # Points on the trunk FRONT surface: near the seeded range from any ring.
        idx = np.flatnonzero(
            cand & (rng <= front + self.cloud_depth_gap) & (rng >= front - self.cloud_depth_gap)
        )
        if idx.size < self.cloud_min_points:
            return None

        azs = az[idx]
        az_centre = float(np.arctan2(np.mean(np.sin(azs)), np.mean(np.cos(azs))))
        r_median = float(np.median(rng[idx]))

        # Lateral extent perpendicular to the trunk bearing.  Percentile-based so
        # a couple of limb hits on the upper rings cannot inflate the width.
        perp_x, perp_y = -math.sin(az_centre), math.cos(az_centre)
        offs = dx[idx] * perp_x + dy[idx] * perp_y
        lo = float(np.percentile(offs, 100.0 - self.cloud_width_percentile))
        hi = float(np.percentile(offs, self.cloud_width_percentile))
        width_m = hi - lo
        if width_m > self.lidar_cluster_max_width_m:
            return None

        # Vertical-column test: bin by height and compare the lateral centre of
        # each level.  A trunk is a cylinder -> almost no drift.  Branches/canopy
        # -> the centres scatter from level to level.
        zs = pts_world[idx, 2]
        z_span = float(zs.max() - zs.min())
        if z_span < self.cloud_min_column_span_m:
            # Everything is at one height: a branch/foliage sheet seen edge-on,
            # not something that stands up like a trunk.
            return None
        bin_m = max(1e-3, self.cloud_level_bin_m)
        levels = np.round((zs - float(zs.min())) / bin_m).astype(np.int64)
        centres = [float(offs[levels == lv].mean()) for lv in np.unique(levels)]
        n_levels = len(centres)
        column_spread = float(np.std(centres)) if n_levels >= 2 else 0.0
        return r_median, az_centre, width_m, n_levels, column_spread

    def _cloud_bush_check(self, pts_world, origin_xy, az_trunk: float, r_trunk: float):
        """Reject a low foliage blob (a BUSH) that YOLO boxed as a tree.

        The detector has no bush class and no negative examples, so bushes come
        through labelled OAK_TREE.  Two independent tests on the full cloud
        separate them:

        1. **No narrow column anywhere.**  A trunk is narrow at EVERY height; a
           bush is wide at every height.  So bin the candidate object by z and,
           at each level, take the CONTIGUOUS lateral run containing the trunk
           bearing and measure its width.  The NARROWEST such cross-section is
           the test value: a real trunk always has a narrow one, while a bush has
           none.  Using the narrowest (not the widest) also keeps a tree's own
           canopy - which is wide and may sit only slightly nearer than the
           trunk - from inflating the measurement and rejecting real trees.
        2. **Trunk vs canopy range.**  A real trunk continues upwards into its
           own canopy, so the nearest surface above the trunk is at a similar
           range (on an oak the low branches are usually slightly NEARER than the
           trunk).  A bush is a low blob: above it there is only sky or much more
           distant geometry.  So when the trunk is much NEARER than whatever is
           above it, the "tree" is really a foreground blob - drop it.

        NOTE the candidate object is gathered in a MUCH wider sector than the
        trunk-search sector (``bush_sector_rad`` vs ``cloud_angle_tolerance``).
        The search sector is only +/-5 deg, which at 5 m is +/-0.45 m of lateral
        extent - far narrower than a 2.5 m bush, so measuring width inside it can
        never see the bush and the test would silently never fire.

        Returns ``(reason, metrics)``; ``reason`` is ``None`` when the object is
        consistent with a trunk.
        """
        metrics = {}
        if pts_world is None or len(pts_world) == 0:
            return None, metrics

        dx = pts_world[:, 0] - origin_xy[0]
        dy = pts_world[:, 1] - origin_xy[1]
        rng = np.hypot(dx, dy)
        ok = (
            np.isfinite(rng)
            & (rng >= self.lidar_range_min)
            & (rng <= self.lidar_range_max)
        )
        if not np.any(ok):
            return None, metrics

        az = np.arctan2(dy, dx)
        diff = np.abs((az - az_trunk + math.pi) % (2.0 * math.pi) - math.pi)
        sector = ok & (diff <= self.bush_sector_rad)
        if not np.any(sector):
            return None, metrics

        # Points belonging to the candidate object.  Two windows on purpose:
        #   * WIDTH test uses only the near face (r_trunk +/- cloud_depth_gap).
        #     A cylinder/bush face is shallow - 2.5 m wide at 5 m range only
        #     spans 0.15 m of depth - so this still contains the whole face while
        #     excluding the tree's own canopy, which for an oak sits NEARER and
        #     would otherwise be measured as "too wide".
        #   * the "what is above" test uses the deeper window so the object's
        #     full vertical extent (its top) is known.
        near = sector & (np.abs(rng - r_trunk) <= self.cloud_depth_gap)
        obj = sector & (rng >= r_trunk - self.cloud_depth_gap) & (
            rng <= r_trunk + self.canopy_probe_depth_m
        )
        if int(np.count_nonzero(near)) < self.cloud_min_points:
            near = obj
        if int(np.count_nonzero(obj)) < self.cloud_min_points:
            return None, metrics

        # Lateral offset perpendicular to the trunk bearing.
        perp_x, perp_y = -math.sin(az_trunk), math.cos(az_trunk)
        offs_all = dx * perp_x + dy * perp_y

        # --- Test 1: width of the contiguous run at each height --------------
        zs = pts_world[:, 2]
        bin_m = max(1e-3, self.cloud_level_bin_m)
        levels = np.round((zs - float(zs[near].min())) / bin_m).astype(np.int64)
        widest = 0.0
        narrowest = float("inf")
        n_levels = 0
        for lv in np.unique(levels[near]):
            sel = near & (levels == lv)
            if not np.any(sel):
                continue
            run = np.sort(offs_all[sel])
            # Contiguous run containing the trunk bearing (offset 0).  Split
            # wherever neighbouring hits are further apart than the gap, so a
            # separate object at another lateral position cannot inflate it.
            breaks = np.flatnonzero(np.diff(run) > self.bush_lateral_gap_m)
            starts = np.concatenate(([0], breaks + 1))
            ends = np.concatenate((breaks, [run.size - 1]))
            best = None
            for s, e in zip(starts, ends):
                if run[s] <= 0.0 <= run[e]:
                    best = float(run[e] - run[s])
                    break
            if best is None:
                # No run straddles the bearing; use the one nearest to it.
                mid = (run[starts] + run[ends]) / 2.0
                k = int(np.argmin(np.abs(mid)))
                best = float(run[ends[k]] - run[starts[k]])
            widest = max(widest, best)
            narrowest = min(narrowest, best)
            n_levels += 1

        if n_levels == 0:
            return None, metrics
        metrics["width"] = narrowest
        metrics["widest"] = widest
        metrics["n_levels"] = n_levels
        if narrowest > self.trunk_max_width_m:
            return (
                f"no narrow column anywhere: narrowest cross-section {narrowest:.1f} m "
                f"(ceiling {self.trunk_max_width_m:.1f} m)",
                metrics,
            )

        # --- Test 2: what is above it? --------------------------------------
        z_top = float(zs[obj].max())
        above = sector & (zs > z_top + self.canopy_gap_m)
        if np.any(above):
            r_above = float(np.median(rng[above]))
            gap = r_above - r_trunk
            metrics["canopy_range"] = r_above
            metrics["canopy_gap"] = gap
            if gap > self.trunk_canopy_max_gap_m:
                return (
                    f"trunk {gap:.1f} m nearer than the surface above it", metrics
                )
        # No returns above the object -> the trunk may simply run out of the
        # lidar's +/-15 deg fan.  Ambiguous, so do not reject on that alone.
        return None, metrics

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
        sensor = self._sensor_in_world(cam_frame)
        if sensor is None:
            return response
        cam_origin, R_wc = sensor

        # Full 3D cloud (all 25 vertical rings), decoded ONCE per request.  With
        # `ring_from_cloud` this is the node's only lidar input: the level ring,
        # the trunk-column test and the bush guard all read these same points.
        cloud_world = self._cloud_points_world()

        lidar_data = self._lidar_scan_data(cloud_world)
        if self.require_lidar_confirmation and lidar_data is None:
            self.get_logger().warn("Lidar confirmation required but no usable scan; returning no locations")
            return response
        lidar_origin = lidar_data[0] if lidar_data else None
        scan_az = lidar_data[1] if lidar_data else None
        scan_ranges = lidar_data[2] if lidar_data else None
        angle_increment = lidar_data[3] if lidar_data else None

        h, w = depth.shape[:2]
        img_h, img_w = self.depth_msg.height, self.depth_msg.width

        kept = 0
        dropped_no_tree = 0
        dropped_no_trunk = 0
        dropped_no_lidar = 0
        dropped_branch_only = 0
        dropped_not_column = 0
        dropped_bush = 0
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

            # Seed the lidar trunk search on the canopy centroid bearing.  For an
            # upright tree the canopy centroid sits directly above the trunk, so
            # this bearing is an unbiased seed and the lidar then refines it to
            # the trunk centre.  The depth image is NOT used for the bearing: in
            # a dense forest its nearest return is a branch, not the trunk.
            az_seed, cam_origin = self._pixel_bearing_world(
                cam_frame, (left + right) // 2, (top + bottom) // 2
            ) or (None, None)
            if az_seed is None or lidar_origin is None:
                dropped_no_lidar += 1
                continue

            # Expected trunk range from the canopy-centroid depth pixel: the
            # canopy sits directly above the trunk, so its horizontal range is a
            # good estimate of the trunk's range.  Seeding the lidar on the echo
            # CLOSEST to this range (not the nearest echo) stops a nearer
            # neighbour trunk/branch from stealing the centroid.
            r_box = None
            pt_box = self._depth_point_camera_frame(
                depth, (left + right) // 2, (top + bottom) // 2
            )
            if pt_box is not None:
                pt_cam = pt_box if is_optical else np.array([pt_box[2], -pt_box[0], -pt_box[1]])
                pt_world = R_wc @ pt_cam + cam_origin
                r_box = math.hypot(pt_world[0] - lidar_origin[0], pt_world[1] - lidar_origin[1])

            # Confirm with a lidar trunk echo at that bearing; this also yields
            # the trunk/cone width (for species verification).
            #
            # The LEVEL RING is tried first: while the drone flies at trunk height
            # it is the most accurate estimator, so cruise-altitude results are
            # unchanged.  The FULL 3D CLOUD is the fallback for when the ring finds
            # nothing - which is exactly what happens as soon as the altitude
            # changes (a duck to 0.8 m passes under the trunk, a pop-up to 8-12 m
            # passes over it).  Its +/-15 deg vertical fan always crosses the trunk.
            r_lidar = az_trunk = width_m = None
            n_levels = 0
            column_spread = 0.0
            anchor_origin = lidar_origin

            cluster = self._lidar_cluster_at(
                lidar_origin, scan_az, scan_ranges, angle_increment, az_seed, r_box
            )
            if cluster is not None:
                r_lidar, az_trunk, width_m, _n_beams = cluster

            if r_lidar is None and cloud_world is not None:
                trunk_cloud = self._cloud_trunk_at(
                    cloud_world[0], cloud_world[1], az_seed, r_box
                )
                if trunk_cloud is not None:
                    r_lidar, az_trunk, width_m, n_levels, column_spread = trunk_cloud
                    # (range, bearing) are relative to the CLOUD's sensor origin.
                    anchor_origin = cloud_world[1]
                    if n_levels >= self.cloud_min_levels and column_spread > self.cloud_column_max_spread_m:
                        # Same bearing at several heights but the lateral centre
                        # moves level to level -> branches/canopy, not a trunk.
                        if self.debug:
                            self.get_logger().info(
                                f"[dbg] {bb.class_name} az_seed={math.degrees(az_seed):.1f}deg "
                                f"cloud column drift {column_spread:.2f} m over {n_levels} levels -> not a trunk")
                        dropped_not_column += 1
                        continue
                    if self.debug:
                        self.get_logger().info(
                            f"[dbg] {bb.class_name} ring missed the trunk; cloud column "
                            f"r={r_lidar:.1f} m over {n_levels} levels (drift {column_spread:.2f} m)")

            if r_lidar is None:
                if self.debug:
                    self.get_logger().info(
                        f"[dbg] {bb.class_name} box({left},{top},{right},{bottom}) "
                        f"az_seed={math.degrees(az_seed):.1f}deg -> no lidar trunk echo (ring or cloud)")
                dropped_no_lidar += 1
                continue

            # Bush guard: a bush is a wide, LOW blob whose top has only distant
            # surfaces above it, whereas a trunk continues up into its canopy.
            # Needs the full 3D cloud, so it is skipped when the cloud is stale.
            if cloud_world is not None:
                bush_reason, bush_metrics = self._cloud_bush_check(
                    cloud_world[0], cloud_world[1], az_trunk, r_lidar
                )
                if bush_reason is not None:
                    if self.debug:
                        self.get_logger().info(
                            f"[dbg] {bb.class_name} az={math.degrees(az_trunk):.1f}deg "
                            f"r={r_lidar:.1f} m -> bush: {bush_reason}"
                        )
                    dropped_bush += 1
                    continue

            # Reject detections whose lidar echo is too wide to be a trunk (an
            # oak showing only branches, trunk hidden) so a centroid is never
            # placed on the branches.  Pines (cones) are allowed wide.
            if not self._trunk_is_credible(bb.class_name, width_m):
                dropped_branch_only += 1
                continue

            # Anchor the centroid on the lidar trunk echo (range + trunk-centre
            # bearing).  This is the lidar's own measurement of the trunk's
            # horizontal position at the lidar plane (search clearance).
            base_x = anchor_origin[0] + r_lidar * math.cos(az_trunk)
            base_y = anchor_origin[1] + r_lidar * math.sin(az_trunk)

            # Base height: prefer the lowest visible trunk row from the depth
            # image (close to the terrain); the horizontal position never
            # depends on it.  Falls back to ground level when no trunk row is
            # visible.
            base_z = 0.0
            trunk = self._find_trunk_column(depth, left, top, right, bottom)
            if trunk is not None:
                _u_c, v_c, _dm, _wp = trunk
                pt_opt = self._depth_point_camera_frame(depth, _u_c, v_c)
                if pt_opt is not None:
                    pt_cam = pt_opt if is_optical else np.array([pt_opt[2], -pt_opt[0], -pt_opt[1]])
                    base_z = float((R_wc @ pt_cam + cam_origin)[2])

            tree = TreeLocation()
            # Species comes from the YOLO detection; the merger majority-votes it
            # across inferences.  Lidar only validated the trunk/centroid.
            tree.tree_type = bb.class_name
            tree.location = Point(x=float(base_x), y=float(base_y), z=base_z)
            tree.score = float(getattr(bb, "score", 1.0))
            tree.trunk_width_m = float(width_m)
            response.tree_data.append(tree)
            kept += 1

        # Per-frame dedup: YOLO can double-box one trunk.  Keep the
        # highest-confidence location within frame_dedup_distance.
        if len(response.tree_data) > 1:
            response.tree_data.sort(key=lambda tr: tr.score, reverse=True)
            deduped = []
            for tree in response.tree_data:
                if all(
                    math.hypot(
                        tree.location.x - k.location.x,
                        tree.location.y - k.location.y,
                    )
                    > self.frame_dedup_distance
                    for k in deduped
                ):
                    deduped.append(tree)
            response.tree_data = deduped
            kept = len(deduped)

        self.get_logger().info(
            f"Processed {len(detections.bounding_boxes)} detection(s) -> {kept} tree location(s) "
            f"({dropped_no_tree} dropped: no tree in box; "
            f"{dropped_no_lidar} dropped: no lidar trunk echo; "
            f"{dropped_not_column} dropped: not a vertical trunk; "
            f"{dropped_bush} dropped: bush (too wide / no canopy above); "
            f"{dropped_branch_only} dropped: branches only, no trunk)"
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
