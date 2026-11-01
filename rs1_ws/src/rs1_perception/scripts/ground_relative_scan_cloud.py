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


def select_obstacles(ground, obstacles, radius, min_clearance, robot_height):
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

    distances, indices = cKDTree(ground[:, :2]).query(
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
    reliable = ((valid.sum(axis=1) >= 6) & (eigenvalues[:, 0] > 0.001) &
                (error < 0.06) & (np.linalg.norm(coefficients[:, :2], axis=1) < 0.7))

    # The fitted intercept is ground Z directly under each candidate point.
    height = obstacles[:, 2] - coefficients[:, 2]
    blocking = (height >= min_clearance) & (height <= robot_height)
    return obstacles[~reliable | blocking]


class GroundRelativeScanCloud(Node):
    def __init__(self):
        super().__init__("ground_relative_scan_cloud")
        self.radius = self.declare_parameter("ground_radius", 0.75).value
        self.clearance = self.declare_parameter("min_clearance", 0.08).value
        self.height = self.declare_parameter("robot_height", 1.0).value
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
                                  self.radius, self.clearance, self.height)
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
