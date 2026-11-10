"""
DBH detector - 3D LiDAR breast-height slice approach.

Pipeline
    /husky1/scan/points (PointCloud2, 17 layers, lidar frame)
        -> numpy XYZ
        -> height above ground per point (z + lidar_height)
        -> keep points at breast height (1.3 m +- tolerance) within range limits
        -> project slice to XY
        -> Euclidean clustering (spatial hash, no extra dependencies)
        -> circle fit per cluster (Kasa + geometric refinement)
        -> validity checks (radius, fit RMS, arc coverage)
        -> DBH = 2 * radius  ->  TreeObservation

The detection logic lives in plain module-level functions (no ROS needed) so it
can be unit-tested with synthetic point clouds.
"""

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# --------------------------------------------------------------------------
# Data containers
# --------------------------------------------------------------------------
class TreeObservation:
    def __init__(self, tree_type, x, y, dbh, distance=None):
        self.tree_type = tree_type
        self.x = x
        self.y = y
        self.dbh = dbh
        self.distance = distance


@dataclass
class DetectionParams:
    # Height of the lidar above the ground (m). <= 0 means "estimate from the
    # ground returns in each cloud". Measure it once and set it explicitly.
    lidar_height: float = -1.0
    breast_height: float = 1.3
    # Layers are discrete (3.75 deg apart), so the slice needs to be wide enough
    # that at least one layer lands inside it. Rule of thumb: tol >= 0.035 * range.
    height_tolerance: float = 0.20
    min_range: float = 0.8
    max_range: float = 5.0
    cluster_distance: float = 0.15
    min_cluster_points: int = 15
    # Trunk validity (known trunk diameters < 1 m)
    min_radius: float = 0.05
    max_radius: float = 0.60
    max_fit_rms: float = 0.04
    # Minimum angular coverage of the fitted circle by the points. A single
    # viewpoint sees at most ~180 deg of a trunk.
    min_arc_deg: float = 90.0
    # A circle's diameter can never be smaller than the longest chord of its own
    # points. Reject fits where 2*r < diameter_slack * (longest point-to-point distance).
    diameter_slack: float = 0.9
    # Log a per-layer / ground-plane diagnostic every N processed clouds (0 = off)
    diagnose_every_n: int = 5


# --------------------------------------------------------------------------
# Pure detection functions (no ROS)
# --------------------------------------------------------------------------
def pointcloud_to_numpy(msg):
    """PointCloud2 -> (N, 3) float64 array of finite XYZ points."""
    cloud = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # Humble returns a structured numpy array; older/other versions return a generator.
    if isinstance(cloud, np.ndarray) and cloud.dtype.names:
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
    else:
        xyz = np.array(list(cloud), dtype=np.float64).reshape(-1, 3)

    xyz = xyz.astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def breast_height_slice(xyz, height_above_ground, p):
    """Return the XY coordinates of points within the breast-height band."""
    horiz = np.hypot(xyz[:, 0], xyz[:, 1])

    mask = (
        (np.abs(height_above_ground - p.breast_height) <= p.height_tolerance)
        & (horiz >= p.min_range)
        & (horiz <= p.max_range)
    )
    return xyz[mask, :2]


def euclidean_cluster(xy, threshold, min_size):
    """
    Region-growing clustering for unordered 2D points. Uses a spatial hash so
    cost is roughly linear in the number of points (the old O(n^2) version
    took >1 s on a few thousand points).
    """
    n = len(xy)
    if n == 0:
        return []

    pts = xy.tolist()
    thr2 = threshold * threshold

    keys = [
        (int(math.floor(x / threshold)), int(math.floor(y / threshold))) for x, y in pts
    ]
    cells = {}
    for idx, key in enumerate(keys):
        cells.setdefault(key, []).append(idx)

    visited = [False] * n
    clusters = []

    for seed in range(n):
        if visited[seed]:
            continue

        visited[seed] = True
        stack = [seed]
        members = []

        while stack:
            i = stack.pop()
            members.append(i)
            xi, yi = pts[i]
            kx, ky = keys[i]

            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in cells.get((kx + dx, ky + dy), ()):
                        if visited[j]:
                            continue
                        xj, yj = pts[j]
                        if (xi - xj) ** 2 + (yi - yj) ** 2 < thr2:
                            visited[j] = True
                            stack.append(j)

        if len(members) >= min_size:
            clusters.append(xy[members])

    return clusters


def fit_circle(xy, iterations=15):
    """
    Least-squares circle fit.
    1) Algebraic (Kasa) fit for a starting point
    2) Gauss-Newton refinement on the geometric residual (dist - r)

    Returns (cx, cy, radius, rms_residual) or None if the fit fails.
    """
    pts = np.asarray(xy, dtype=np.float64)
    if len(pts) < 3:
        return None

    mean = pts.mean(axis=0)
    x = pts[:, 0] - mean[0]
    y = pts[:, 1] - mean[1]

    A = np.column_stack((2.0 * x, 2.0 * y, np.ones(len(x))))
    b = x * x + y * y

    try:
        sol = np.linalg.lstsq(A, b, rcond=None)[0]
    except np.linalg.LinAlgError:
        return None

    cx, cy, c = sol
    r2 = c + cx * cx + cy * cy
    if r2 <= 0.0:
        return None
    r = math.sqrt(r2)

    for _ in range(iterations):
        dx = x - cx
        dy = y - cy
        dist = np.maximum(np.hypot(dx, dy), 1e-9)
        res = dist - r

        J = np.column_stack((-dx / dist, -dy / dist, -np.ones(len(x))))

        try:
            step = np.linalg.lstsq(J, -res, rcond=None)[0]
        except np.linalg.LinAlgError:
            break

        cx += step[0]
        cy += step[1]
        r += step[2]

        if np.max(np.abs(step)) < 1e-6:
            break

    if r <= 0.0:
        return None

    res = np.hypot(x - cx, y - cy) - r
    rms = float(np.sqrt(np.mean(res * res)))

    return float(cx + mean[0]), float(cy + mean[1]), float(r), rms


def arc_span_deg(xy, cx, cy):
    """Angular coverage (deg) of the points around the fitted centre."""
    angles = np.sort(np.arctan2(xy[:, 1] - cy, xy[:, 0] - cx))
    gaps = np.diff(np.concatenate((angles, [angles[0] + 2.0 * math.pi])))
    return math.degrees(2.0 * math.pi - gaps.max())


def max_extent(xy, cap=600):
    """Longest point-to-point distance in a cluster (subsampled if very large)."""
    pts = xy if len(xy) <= cap else xy[np.linspace(0, len(xy) - 1, cap).astype(int)]
    diff = pts[:, None, :] - pts[None, :, :]
    return float(np.sqrt((diff**2).sum(axis=-1)).max())


def evaluate_cluster(cluster, p):
    """Fit a circle to one cluster and decide if it looks like a trunk."""
    xs = cluster[:, 0]
    ys = cluster[:, 1]

    result = {
        "accepted": False,
        "reason": "",
        "n": len(cluster),
        "wx": float(xs.max() - xs.min()),
        "wy": float(ys.max() - ys.min()),
        "distance": float(np.min(np.hypot(xs, ys))),
        "mx": float(xs.mean()),  # centroid of the points (for logging)
        "my": float(ys.mean()),
        "cx": float(xs.mean()),  # fitted circle centre (overwritten below)
        "cy": float(ys.mean()),
        "extent": max_extent(cluster),
        "radius": None,
        "dbh": None,
        "rms": None,
        "arc_deg": None,
    }

    fit = fit_circle(cluster)
    if fit is None:
        result["reason"] = "circle fit failed"
        return result

    cx, cy, r, rms = fit
    arc = arc_span_deg(cluster, cx, cy)

    result.update(cx=cx, cy=cy, radius=r, dbh=2.0 * r, rms=rms, arc_deg=arc)

    if not (p.min_radius <= r <= p.max_radius):
        result["reason"] = f"radius {r:.2f} outside [{p.min_radius}, {p.max_radius}]"
    elif rms > p.max_fit_rms:
        result["reason"] = f"fit rms {rms:.3f} > {p.max_fit_rms}"
    elif arc < p.min_arc_deg:
        result["reason"] = f"arc {arc:.0f} deg < {p.min_arc_deg:.0f}"
    elif 2.0 * r < p.diameter_slack * result["extent"]:
        result["reason"] = (
            f"fit diameter {2.0 * r:.2f} < cluster extent {result['extent']:.2f}"
        )
    else:
        result["accepted"] = True
        result["reason"] = "ok"

    return result


LAYER_COUNT = 17
LAYER_MIN_DEG = -30.0
LAYER_STEP_DEG = 3.75


def layer_index(xyz):
    """Layer number (0..16) of each point, from its elevation angle in the sensor frame."""
    horiz = np.hypot(xyz[:, 0], xyz[:, 1])
    elev = np.degrees(np.arctan2(xyz[:, 2], horiz))
    return np.rint((elev - LAYER_MIN_DEG) / LAYER_STEP_DEG).astype(int)


def fit_ground_plane(xyz, min_horiz=0.6, max_horiz=8.0, max_layer=2, min_points=100):
    """
    Robust ground plane z = a*x + b*y + c in the lidar frame, fitted to returns
    from the steepest downward layers.

    - min_horiz drops self-returns from the Husky body (lidar min range is 0.2 m)
    - a few fit / reject-outlier passes drop trunks and bumps that sneak in

    Returns (a, b, c, n_used) or None.
    """
    horiz = np.hypot(xyz[:, 0], xyz[:, 1])
    cand = (layer_index(xyz) <= max_layer) & (horiz >= min_horiz) & (horiz <= max_horiz)
    if cand.sum() < min_points:
        return None

    pts = xyz[cand]
    A = np.column_stack((pts[:, 0], pts[:, 1], np.ones(len(pts))))
    keep = np.ones(len(pts), dtype=bool)
    coef = None

    for _ in range(4):
        if keep.sum() < min_points:
            return None
        coef = np.linalg.lstsq(A[keep], pts[keep, 2], rcond=None)[0]
        keep = np.abs(pts[:, 2] - A @ coef) < 0.15

    if keep.sum() < min_points:
        return None

    a, b, c = coef
    return float(a), float(b), float(c), int(keep.sum())


def height_above_plane(xyz, plane):
    a, b, c = plane[:3]
    return (xyz[:, 2] - a * xyz[:, 0] - b * xyz[:, 1] - c) / math.sqrt(
        1 + a * a + b * b
    )


def diagnose_cloud(xyz, p, near_range=5.0):
    """
    Explain why a breast-height slice is empty. Returns a list of log lines.

    1) Per-layer stats (layer index from elevation angle, which is fixed in the
       sensor frame so it is unaffected by robot tilt).
    2) Ground plane fitted to the steepest downward layers near the robot
       -> tilt, perpendicular lidar height.
    3) Slice counts using (a) your constant lidar_height and (b) height above
       the fitted ground plane.
    """
    lines = []
    horiz = np.hypot(xyz[:, 0], xyz[:, 1])
    elev = np.degrees(np.arctan2(xyz[:, 2], horiz))
    layer = np.rint((elev - LAYER_MIN_DEG) / LAYER_STEP_DEG).astype(int)

    lines.append("layer elev(deg)    n  nearest_r  median_z(r<=%.0fm)" % near_range)
    for k in range(LAYER_COUNT):
        m = layer == k
        n = int(m.sum())
        e = LAYER_MIN_DEG + k * LAYER_STEP_DEG
        if n == 0:
            lines.append(f"{k:5d} {e:8.2f} {0:4d}        -          -")
            continue
        near = m & (horiz <= near_range)
        med = f"{np.median(xyz[near, 2]):9.2f}" if near.any() else "        -"
        lines.append(f"{k:5d} {e:8.2f} {n:4d} {horiz[m].min():10.2f} {med}")

    plane = fit_ground_plane(xyz)
    if plane is not None:
        a, b, c, n_used = plane
        norm = math.sqrt(1 + a * a + b * b)
        tilt = math.degrees(math.atan(math.hypot(a, b)))
        lines.append(
            f"Ground plane (n={n_used}, self-returns excluded): tilt={tilt:.1f} deg, "
            f"lidar height above plane={-c / norm:.2f} m  [param lidar_height={p.lidar_height:.2f}]"
        )
        hag = height_above_plane(xyz, plane)
    else:
        lines.append("Ground plane: not enough ground returns (layers 0-2, 0.6-8 m)")
        hag = None

    in_range = (horiz >= p.min_range) & (horiz <= p.max_range)
    if p.lidar_height > 0.0:
        const_h = xyz[:, 2] + p.lidar_height
        n_const = int(
            (in_range & (np.abs(const_h - p.breast_height) <= p.height_tolerance)).sum()
        )
        lines.append(
            f"Slice with constant lidar_height={p.lidar_height:.2f}: {n_const} pts"
        )

    if hag is not None:
        n_plane = int(
            (in_range & (np.abs(hag - p.breast_height) <= p.height_tolerance)).sum()
        )
        obst = in_range & (hag > 0.3)
        nearest = f"{horiz[obst].min():.2f} m" if obst.any() else "none"
        any_obst = (horiz >= p.min_range) & (hag > 0.3)
        if any_obst.any():
            idx = np.where(any_obst)[0][np.argmin(horiz[any_obst])]
            lines.append(
                f"Nearest non-ground return (any range): {horiz[idx]:.2f} m, "
                f"{hag[idx]:.2f} m above ground  [max_range={p.max_range:.1f}]"
            )
        else:
            lines.append("Nearest non-ground return (any range): none")
        lines.append(
            f"Slice with ground-plane height: {n_plane} pts | "
            f"non-ground pts (>0.3 m above plane) in range: {int(obst.sum())}, nearest {nearest}"
        )
        # height histogram of in-range points above ground plane
        edges = np.arange(-0.5, 3.01, 0.5)
        counts, _ = np.histogram(hag[in_range], bins=edges)
        lines.append(
            "Height-above-plane histogram (0.5 m bins from -0.5): "
            + " ".join(map(str, counts.tolist()))
        )

    return lines


def detect_trees(xyz, p, *, target_xy=None, center_tolerance=0.35):
    """
    Full detection on one cloud.

    lidar_height > 0  -> fixed height above ground (z + lidar_height)
    lidar_height <= 0 -> height above a ground plane fitted to this cloud

    Returns (lidar_height_used, slice_xy, [cluster result dicts]).
    """
    if p.lidar_height > 0.0:
        lidar_h = p.lidar_height
        height = xyz[:, 2] + lidar_h
    else:
        plane = fit_ground_plane(xyz)
        if plane is None:
            return None, np.empty((0, 2)), []
        a, b, c = plane[:3]
        lidar_h = -c / math.sqrt(1 + a * a + b * b)
        height = height_above_plane(xyz, plane)

    slice_xy = breast_height_slice(xyz, height, p)
    if target_xy is not None:
        # Keep enough bark around the expected centre to fit the whole visible arc
        offset = slice_xy - target_xy
        slice_xy = slice_xy[np.hypot(offset[:, 0], offset[:, 1]) <= center_tolerance + p.max_radius]
    clusters = euclidean_cluster(slice_xy, p.cluster_distance, p.min_cluster_points)
    results = [evaluate_cluster(c, p) for c in clusters]

    return lidar_h, slice_xy, results


@dataclass
class RingWidth:
    layer: int
    height: float
    centre: np.ndarray
    diameter: float


def measure_ring(xy, angles, heights, layer, params):
    half_angle = (angles[-1] - angles[0]) / 2
    if not 0 < half_angle < np.pi / 3:
        return None
    ends = xy[[0, -1]]
    direction = (ends / np.linalg.norm(ends, axis=1)[:, None]).sum(axis=0)
    direction /= np.linalg.norm(direction)
    side = np.array([-direction[1], direction[0]])
    across, depth = xy @ side, xy @ direction
    middle = (across.min() + across.max()) / 2
    front = np.percentile(depth, 10)

    # The tangent points sit just inside the full width when we're close to the trunk
    beam = front * math.radians(0.25)
    diameter = (np.ptp(across) + beam) / math.cos(half_angle)
    if not 2 * params.min_radius <= diameter <= 2 * params.max_radius:
        return None

    # Compare the middle to the chord so a tilted flat surface doesn't look like a trunk
    edge = max(2, len(depth) // 10)
    inside = np.abs(across - middle) < diameter / 8
    if not inside.any():
        return None
    chord_depth = np.interp(np.median(across[inside]),
                            [np.median(across[:edge]), np.median(across[-edge:])],
                            [np.median(depth[:edge]), np.median(depth[-edge:])])
    if chord_depth - np.median(depth[inside]) < 0.035 * diameter:
        return None
    centre = direction * (front + diameter / 2) + side * middle
    # On a small round stem the last laser beam often lands short of the silhouette
    if diameter < 0.45:
        fit = fit_circle(xy)
        if fit is not None:
            cx, cy, radius, rms = fit
            if (abs(2 * radius - diameter) < 0.15 * diameter and rms < 0.015 * diameter
                    and arc_span_deg(xy, cx, cy) >= params.min_arc_deg):
                centre = np.array([cx, cy])
                diameter = 2 * radius
    return RingWidth(layer, float(np.median(heights)), centre, diameter)


def trunk_widths(xyz, p, target, tolerance, locating=False, diagnostics=None):
    """Measure the silhouette of standing trunks across individual LiDAR rings."""
    if p.lidar_height > 0:
        heights = xyz[:, 2] + p.lidar_height
    else:
        plane = fit_ground_plane(xyz)
        if diagnostics is not None:
            diagnostics['ground_plane'] = None if plane is None else np.asarray(plane).tolist()
        if plane is None:
            return None
        heights = height_above_plane(xyz, plane)

    # Keep full silhouettes around the search area, including the far edge of a broad trunk
    lowest_ring = 0.25 if locating else 0.50
    mask = ((np.linalg.norm(xyz[:, :2] - target, axis=1) < tolerance + 2 * p.max_radius)
            & (heights > lowest_ring) & (heights < 2.0))
    points, heights = xyz[mask], heights[mask]
    layers = layer_index(points)
    angles = np.arctan2(points[:, 1], points[:, 0]) - math.atan2(target[1], target[0])
    angles = (angles + np.pi) % (2 * np.pi) - np.pi
    distances = np.linalg.norm(points[:, :2], axis=1)
    tracks = []
    if diagnostics is not None:
        diagnostics.update(cropped_points=len(points), rings=[], tracks=[], invalid_ring_groups=0)
    for layer in np.unique(layers):
        indices = np.flatnonzero(layers == layer)
        indices = indices[np.argsort(angles[indices])]
        gaps = ((np.diff(angles[indices]) > math.radians(0.65))
                | (np.abs(np.diff(distances[indices])) > 0.12))
        for group in np.split(indices, np.flatnonzero(gaps) + 1):
            if len(group) < 5:
                continue
            ring = measure_ring(points[group, :2], angles[group], heights[group], int(layer), p)
            if diagnostics is not None:
                if ring is None:
                    diagnostics['invalid_ring_groups'] += 1
                else:
                    diagnostics['rings'].append(dict(layer=ring.layer, height=ring.height,
                        centre=ring.centre.tolist(), diameter=float(ring.diameter),
                        offset=float(np.linalg.norm(ring.centre-target))))
            if ring is None or np.linalg.norm(ring.centre - target) > tolerance:
                continue
            centre, diameter = ring.centre, ring.diameter
            # Follow the same stem up from below breast height so branches don't merge nearby trees
            matches = [t for t in tracks if 0 < layer - t[-1].layer <= 2 and
                       np.linalg.norm(centre-t[-1].centre) < 0.13 and
                       abs(diameter-t[-1].diameter) < max(0.045, 0.22*t[-1].diameter)]
            if matches:
                min(matches, key=lambda t: np.linalg.norm(centre-t[-1].centre)).append(ring)
            else:
                tracks.append([ring])

    stems = []
    for track in tracks:
        trace = dict(heights=[r.height for r in track], diameters=[float(r.diameter) for r in track],
                     centres=[r.centre.tolist() for r in track], accepted=False)
        if diagnostics is not None:
            diagnostics['tracks'].append(trace)
        # Foliage around a metre up can mimic a wide trunk across several rings
        # Only follow a track that also reaches the lower stem
        if min(r.height for r in track) > 0.75:
            trace['reason'] = 'No supporting lower stem'
            continue
        if locating:
            # A lower section can guide the approach even if branches hide breast height
            lower = [r for r in track if r.height <= p.breast_height]
            if len(lower) >= 2 and np.ptp([r.height for r in lower]) >= 0.12:
                trace['accepted'] = True
                stems.append((np.median([r.diameter for r in lower]),
                              np.median([r.centre for r in lower], axis=0)))
            continue
        if len(track) < 3 or min(r.height for r in track) > p.breast_height - 0.16:
            trace['reason'] = 'Insufficient rings below breast height'
            continue
        near = sorted(track, key=lambda r: abs(r.height - p.breast_height))
        if abs(near[0].height - p.breast_height) > p.height_tolerance:
            trace['reason'] = 'No ring near breast height'
            continue
        # Interpolate to breast height between rings instead of fitting a circle to a thick slice
        ordered = sorted(track, key=lambda r: r.height)
        sizes = [r.diameter for r in ordered]
        diameter = float(np.interp(p.breast_height, [r.height for r in ordered], sizes))
        stems.append((diameter, np.median([r.centre for r in near[:2]], axis=0)))
        trace['accepted'] = True
    return stems


class TreeTracker:
    """
    Groups per-frame detections of the same tree (nearest centre within `gate`)
    and keeps the last `history` DBH values per tree, so the reported DBH is a
    median rather than a single noisy frame.

    Assumes the robot is roughly stationary while measuring, or that x, y are
    already in a fixed (global) frame. If the robot moves and x, y are local,
    tracks simply restart.
    """

    def __init__(self, gate=0.4, history=15, max_missed=10):
        self.gate = gate
        self.history = history
        self.max_missed = max_missed
        self.tracks = []
        self.next_id = 0

    def update(self, detections):
        """detections: list of (x, y, dbh). Returns summaries of tracks seen this frame."""
        for t in self.tracks:
            t["missed"] += 1
            t["seen"] = False

        for x, y, dbh in detections:
            best, best_d = None, self.gate
            for t in self.tracks:
                if t["seen"]:
                    continue
                d = math.hypot(t["x"] - x, t["y"] - y)
                if d < best_d:
                    best, best_d = t, d

            if best is None:
                best = {
                    "id": self.next_id,
                    "x": x,
                    "y": y,
                    "dbh": [],
                    "missed": 0,
                    "seen": False,
                }
                self.next_id += 1
                self.tracks.append(best)

            best["x"] = 0.8 * best["x"] + 0.2 * x
            best["y"] = 0.8 * best["y"] + 0.2 * y
            best["dbh"] = (best["dbh"] + [dbh])[-self.history :]
            best["missed"] = 0
            best["seen"] = True

        self.tracks = [t for t in self.tracks if t["missed"] <= self.max_missed]

        out = []
        for t in self.tracks:
            if t["seen"]:
                vals = np.array(t["dbh"])
                out.append(
                    {
                        "id": t["id"],
                        "x": t["x"],
                        "y": t["y"],
                        "n": len(vals),
                        "median": float(np.median(vals)),
                        "spread": float(vals.max() - vals.min()),
                    }
                )
        return out


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------
class DBHDetector(Node):
    def __init__(self):

        super().__init__("dbh_detector")

        defaults = DetectionParams()

        self.declare_parameter("cloud_topic", "/husky1/scan/points")
        self.declare_parameter("process_every_n", 1)
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)

        self.params = self.read_params()
        self.process_every_n = int(self.get_parameter("process_every_n").value)
        cloud_topic = self.get_parameter("cloud_topic").value

        self.cloud_count = 0
        self.processed_count = 0
        self.first_cloud_logged = False
        self.tracker = TreeTracker()

        # Future:
        # populated from TreeLocation[] messages generated by Parrot
        self.tree_locations = []

        self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, qos_profile_sensor_data
        )

        self.get_logger().info(f"DBH Detector started, listening on {cloud_topic}")
        self.get_logger().info(f"Params: {self.params}")

    def read_params(self):
        values = {}
        for name in vars(DetectionParams()).keys():
            values[name] = self.get_parameter(name).value
        return DetectionParams(**values)

    def cloud_callback(self, msg):

        self.cloud_count += 1

        if self.cloud_count % self.process_every_n != 0:
            return

        self.processed_count += 1
        xyz = pointcloud_to_numpy(msg)

        if xyz.shape[0] == 0:
            self.get_logger().warn("Received an empty point cloud")
            return

        if not self.first_cloud_logged:
            self.first_cloud_logged = True
            self.get_logger().info(
                f"First cloud: frame_id={msg.header.frame_id} "
                f"width={msg.width} height={msg.height}"
            )

        self.get_logger().info(
            f"Cloud points={xyz.shape[0]} "
            f"z_min={xyz[:, 2].min():.2f} z_max={xyz[:, 2].max():.2f}"
        )

        lidar_h, slice_xy, results = detect_trees(xyz, self.params)

        if lidar_h is None:
            self.get_logger().warn("Could not determine lidar height")
            return

        if self.params.lidar_height <= 0.0:
            self.get_logger().info(
                f"Estimated lidar height = {lidar_h:.2f} m (fitted ground plane)"
            )

        self.get_logger().info(f"Breast-height slice points = {len(slice_xy)}")

        n_diag = self.params.diagnose_every_n
        if n_diag > 0 and (self.processed_count - 1) % n_diag == 0:
            self.get_logger().info(
                "Diagnostics:\n" + "\n".join(diagnose_cloud(xyz, self.params))
            )
        self.get_logger().info(f"Clusters = {len(results)}")

        for i, res in enumerate(results):
            if res["radius"] is None:
                self.get_logger().info(f"Cluster {i}: n={res['n']} {res['reason']}")
                continue

            self.get_logger().info(
                f"Cluster {i}: "
                f"Points={res['n']} "
                f"Centroid=({res['mx']:.2f},{res['my']:.2f}) "
                f"Ext={res['extent']:.2f} "
                f"R={res['radius']:.3f} "
                f"RMS={res['rms']:.3f} "
                f"Arc={res['arc_deg']:.0f}deg "
                f"-> {'ACCEPT' if res['accepted'] else 'reject: ' + res['reason']}"
            )

        accepted = [r for r in results if r["accepted"]]
        self.get_logger().info(f"Tree candidates = {len(accepted)}")

        detections = []

        for res in accepted:
            # Future:
            # convert centre from local Husky frame into global map frame.
            global_x, global_y = self.local_to_global(res["cx"], res["cy"])

            observation = TreeObservation(
                tree_type="unknown",
                x=global_x,
                y=global_y,
                dbh=res["dbh"],
                distance=res["distance"],
            )

            self.get_logger().info(
                f"Observation: "
                f"type={observation.tree_type}, "
                f"x={observation.x:.2f}, "
                f"y={observation.y:.2f}, "
                f"dbh={observation.dbh:.2f}, "
                f"dist={observation.distance:.2f}"
            )

            detections.append((observation.x, observation.y, observation.dbh))

        for t in self.tracker.update(detections):
            self.get_logger().info(
                f"Tree T{t['id']}: DBH median={t['median']:.2f} m "
                f"(n={t['n']}, spread={t['spread']:.2f}) at ({t['x']:.2f}, {t['y']:.2f})"
            )

    def match_tree_location(self, cx, cy, tree_x, tree_y):

        error = math.sqrt((cx - tree_x) ** 2 + (cy - tree_y) ** 2)

        return error

    def local_to_global(self, local_x, local_y):
        # TODO:
        # Use Husky pose to convert local cluster coordinates into the global map frame.

        return local_x, local_y

    def estimate_agb(self, tree_type, dbh):
        # Placeholder.
        # Species-specific equations
        # added after confirmation.

        if tree_type == "oak":
            return 0.0

        elif tree_type == "pine":
            return 0.0

        return None


def main():

    rclpy.init()

    node = DBHDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
