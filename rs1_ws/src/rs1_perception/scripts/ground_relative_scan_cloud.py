#!/usr/bin/env python3
"""Select SLAM landmarks by clearance above nearby observed ground."""
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField


def points_from_cloud(cloud):
    # An empty ground output is valid when segmentation cannot find a surface.
    # Do not construct strided views with field offsets into its empty buffer.
    if cloud.width * cloud.height == 0:
        return np.empty((0, 3), dtype=np.float32)
    fields = {field.name: field.offset for field in cloud.fields}
    dtype = ">f4" if cloud.is_bigendian else "<f4"
    points = np.column_stack([
        np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                   offset=fields[name], strides=(cloud.row_step, cloud.point_step)).ravel()
        for name in ("x", "y", "z")])
    return points[np.isfinite(points).all(axis=1)]


def select_obstacles(ground, obstacles, radius, min_clearance, robot_height,
                     fallback_radius=1.5, *, height_bound_enabled=True,
                     bound_max_slope=0.7, bound_margin=0.2):
    """Fit small ground patches; keep points whose ground height is uncertain.

    Segmentation can leave a few ground fragments labelled as obstacles at grid
    boundaries. A local plane rejects these without using a fixed robot-relative
    Z cutoff, which would also hide real obstacles farther up a hill.
    """
    if len(ground) < 6 or len(obstacles) == 0:
        return obstacles

    # Spread the ground samples before fitting a surface. Otherwise the nearest
    # 12 points can all come from one dense LiDAR ring, which only describes a
    # line even when other rings have observed the surrounding ground.
    # Average each 20 cm XY square into one sample. This only changes the ground
    # used for fitting; obstacle candidates and the costmap resolution are unchanged.
    ground_cells = np.floor(ground[:, :2] / 0.20)
    _, cell_indices, cell_counts = np.unique(
        ground_cells, axis=0, return_inverse=True, return_counts=True)
    ground = np.column_stack([
        np.bincount(cell_indices, weights=ground[:, axis]) / cell_counts
        for axis in range(3)])

    # Dense points in a tiny patch still do not provide enough independent ground
    # samples. Keep the candidates if grouping leaves too little support.
    if len(ground) < 6:
        return obstacles

    ground_tree = cKDTree(ground[:, :2])
    ground_z, reliable, upper_bound = estimate_ground_height(
        ground, ground_tree, obstacles, radius, bound_max_slope, bound_margin)

    # Only look farther out where the nearby ground fit failed
    # Keep a good local fit even if the wider patch describes different terrain
    missing = np.flatnonzero(~reliable)
    if fallback_radius > radius and len(missing):
        fallback_z, fallback_reliable, fallback_bound = estimate_ground_height(
            ground, ground_tree, obstacles[missing], fallback_radius,
            bound_max_slope, bound_margin)
        ground_z[missing] = fallback_z
        reliable[missing] = fallback_reliable
        upper_bound[missing] = fallback_bound

    # A thin strip of ground cannot tell us the slope in every direction
    # Only remove a point if it clears even the highest ground we allow here
    overhead = height_bound_enabled & (obstacles[:, 2] > upper_bound + robot_height)
    height = obstacles[:, 2] - ground_z
    blocking = (height >= min_clearance) & (height <= robot_height)
    return obstacles[(reliable & blocking) | (~reliable & ~overhead)]


def fit_ground_height(ground, ground_tree, obstacles, radius):
    """Estimate ground height with the same checks at either search radius."""
    height, reliable, _ = estimate_ground_height(ground, ground_tree, obstacles, radius)
    return height, reliable


def estimate_ground_height(ground, ground_tree, obstacles, radius,
                           bound_max_slope=0.7, bound_margin=0.2):
    """Return a plane estimate, its validity and an optional upper height bound

    The bound is only available when horizontal spread is the sole failed check
    It assumes the unseen terrain rises no faster than bound_max_slope, with an
    extra bound_margin for uncertainty, and never declares unseen space free
    """
    distances, indices = ground_tree.query(
        obstacles[:, :2], k=12, distance_upper_bound=radius)
    valid = np.isfinite(distances)
    # Missing neighbours have index len(ground). Use a harmless index and zero
    # weight so they cannot influence the fitted surface.
    neighbours = ground[np.minimum(indices, len(ground) - 1)]
    offsets = neighbours[:, :, :2] - obstacles[:, None, :2]
    design = np.concatenate((offsets, np.ones((*offsets.shape[:2], 1))), axis=2)
    weights = np.where(valid, 1.0 / (distances**2 + 0.01), 0.0)
    normal = np.einsum("nki,nk,nkj->nij", design, weights, design)
    rhs = np.einsum("nki,nk,nk->ni", design, weights, neighbours[:, :, 2])
    coefficients = np.linalg.solve(normal + np.diag([1e-4, 1e-4, 1e-6]), rhs)

    # A single LiDAR ring can be nearly a line. Do not extrapolate a plane from
    # it: require neighbours spread in both horizontal directions and a good fit.
    totals = weights.sum(axis=1)
    safe_totals = np.maximum(totals, 1e-9)
    centre = np.einsum("nki,nk->ni", offsets, weights) / safe_totals[:, None]
    spread = offsets - centre[:, None, :]
    covariance = np.einsum("nki,nk,nkj->nij", spread, weights, spread) / safe_totals[:, None, None]
    eigenvalues = np.linalg.eigvalsh(covariance)
    residuals = np.einsum("nki,ni->nk", design, coefficients) - neighbours[:, :, 2]
    error = np.sqrt((weights * residuals**2).sum(axis=1) / safe_totals)
    supported = valid.sum(axis=1) >= 6
    spread_ok = eigenvalues[:, 0] > 0.001
    error_ok = error < 0.06
    slope_ok = np.linalg.norm(coefficients[:, :2], axis=1) < 0.7
    reliable = supported & spread_ok & error_ok & slope_ok

    # Keep the old checks intact and only handle a consistent but narrow ground strip
    # Use the highest bound from all neighbours so a low sample cannot clear a point
    eligible = supported & ~spread_ok & error_ok & slope_ok
    possible_ground = neighbours[:, :, 2] + bound_max_slope * np.where(valid, distances, 0.0)
    upper_bound = np.max(np.where(valid, possible_ground, -np.inf), axis=1) + bound_margin
    upper_bound = np.where(eligible, upper_bound, np.inf)

    # The fitted intercept is ground Z directly under each candidate point
    return coefficients[:, 2], reliable, upper_bound


class GroundRelativeScanCloud(Node):
    def __init__(self):
        super().__init__("ground_relative_scan_cloud")
        self.radius = self.declare_parameter("ground_radius", 0.75).value
        self.fallback_radius = self.declare_parameter("ground_fallback_radius", 1.5).value
        self.clearance = self.declare_parameter("min_clearance", 0.08).value
        self.height = self.declare_parameter("robot_height", 1.0).value
        # Keep the tested fallback opt-in until full-mission navigation improves too
        self.height_bound_enabled = self.declare_parameter("ground_height_bound_enabled", False).value
        self.bound_max_slope = self.declare_parameter("ground_bound_max_slope", 0.7).value
        self.bound_margin = self.declare_parameter("ground_bound_margin", 0.2).value
        if (not np.isfinite(self.bound_max_slope) or self.bound_max_slope < 0.7 or
                not np.isfinite(self.bound_margin) or self.bound_margin < 0.0):
            raise ValueError("Ground bound needs a finite slope of at least 0.7 and a nonnegative margin")
        self.publisher = self.create_publisher(PointCloud2, "navigation_points", 10)
        self.ground = message_filters.Subscriber(self, PointCloud2, "ground_points")
        self.obstacles = message_filters.Subscriber(self, PointCloud2, "obstacle_points")
        # Both outputs come from one segmentation callback and have the same stamp.
        self.sync = message_filters.TimeSynchronizer([self.ground, self.obstacles], 10)
        self.sync.registerCallback(self.process)

    def process(self, ground, obstacles):
        if ground.header.frame_id != obstacles.header.frame_id:
            self.get_logger().warning("Ground and obstacle frames differ; skipping cloud")
            return
        points = select_obstacles(points_from_cloud(ground), points_from_cloud(obstacles),
                                  self.radius, self.clearance, self.height,
                                  self.fallback_radius,
                                  height_bound_enabled=self.height_bound_enabled,
                                  bound_max_slope=self.bound_max_slope,
                                  bound_margin=self.bound_margin)
        output = PointCloud2()
        output.header = obstacles.header
        output.height = 1
        output.width = len(points)
        output.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                         for index, name in enumerate(("x", "y", "z"))]
        output.point_step = 12
        output.row_step = output.width * output.point_step
        output.is_dense = True
        output.data.frombytes(points.astype("<f4").tobytes())
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = GroundRelativeScanCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
