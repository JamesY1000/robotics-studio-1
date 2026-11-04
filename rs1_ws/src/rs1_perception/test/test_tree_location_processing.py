import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "tree_location" / "tree_location_processing.py"
spec = importlib.util.spec_from_file_location("tree_location_processing", MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
TreeLocationNode = module.TreeLocationNode


def test_find_trunk_column_locates_trunk_centre():
    node = TreeLocationNode.__new__(TreeLocationNode)
    node.trunk_lower_fraction = 0.35
    node.min_depth = 0.4
    node.max_depth = 40.0
    node.trunk_depth_gap = 0.5
    node.trunk_min_width_frac = 0.05
    node.trunk_max_width_frac = 0.6

    depth = np.full((40, 40), 8.0, dtype=np.float32)
    depth[26:40, 18:23] = 2.0  # narrow near vertical column in the lower band

    trunk = node._find_trunk_column(depth, 0, 0, 39, 39)
    assert trunk is not None
    u_center, v_base, depth_med, width_px = trunk
    assert u_center == 20  # middle of columns 18..22
    assert abs(depth_med - 2.0) < 1e-6
    assert width_px == 5


def test_process_callback_rejects_depth_only_when_lidar_cannot_match():
    node = TreeLocationNode.__new__(TreeLocationNode)
    node.hfov = 2.0944
    node.get_logger = lambda: type("L", (), {"info": lambda *args, **kwargs: None, "warn": lambda *args, **kwargs: None})()
    node.trunk_lower_fraction = 0.35
    node.depth_patch = 1
    node.min_depth = 0.4
    node.max_depth = 40.0
    node.min_box_pixels = 1
    node.require_foliage_check = False
    node.require_trunk_check = False
    node.require_lidar_confirmation = False
    node.max_range_difference = 3.0
    node.lidar_angle_tolerance = 0.03
    node.lidar_range_min = 0.2
    node.lidar_range_max = 40.0
    node.world_frame = "world"
    node.lidar_frame = "parrot1_base_scan"
    node.depth_optical_frame = "parrot1_camera_depth_optical_frame"
    node.trunk_depth_gap = 0.5
    node.trunk_min_width_frac = 0.05
    node.trunk_max_width_frac = 0.6
    node.lidar_cluster_depth_gap = 0.3
    node.lidar_cluster_min_beams = 2
    node.lidar_cluster_max_width_m = 3.0
    node.frame_dedup_distance = 1.0
    node.oak_trunk_max_width_m = 0.8
    node._decode_depth = staticmethod(lambda msg: np.full((80, 80), 3.0, dtype=np.float32))
    node._box_shows_foliage = lambda *args, **kwargs: True
    node._lookup = lambda *args, **kwargs: type(
        "Tf",
        (),
        {"transform": type("Transform", (), {"translation": type("T", (), {"x": 0.0, "y": 0.0, "z": 0.0})(), "rotation": type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()})()},
    )()
    node._lidar_scan_data = lambda: (np.array([0.0, 0.0]), np.array([0.0]), np.array([10.0]), 0.01745)
    node._lidar_cluster_at = lambda *args, **kwargs: None

    depth_msg = type(
        "DepthMsg",
        (),
        {"header": type("Header", (), {"frame_id": "parrot1_camera_depth_optical_frame"})(), "height": 80, "width": 80, "encoding": "32FC1", "data": b""},
    )()
    depth_msg.data = np.zeros((80, 80), dtype=np.float32).tobytes()
    node.depth_msg = depth_msg

    class Box:
        class_name = "PINE_TREE"
        score = 0.9
        left = 0
        top = 0
        right = 79
        bottom = 79

    req = type("Req", (), {"start_lidar": True, "detections": type("Detections", (), {"bounding_boxes": [Box()]})()})()
    response = type("Resp", (), {"tree_data": []})()

    node.process_callback(req, response)

    assert response.tree_data == []


def _make_trunk_node():
    node = TreeLocationNode.__new__(TreeLocationNode)
    node.trunk_lower_fraction = 0.35
    node.min_depth = 0.4
    node.max_depth = 40.0
    node.trunk_depth_gap = 0.5
    node.trunk_min_width_frac = 0.05
    node.trunk_max_width_frac = 0.6
    return node


def test_box_has_trunk_detects_narrow_near_column():
    node = _make_trunk_node()
    depth = np.full((40, 40), 8.0, dtype=np.float32)
    depth[26:40, 18:23] = 2.0  # narrow near vertical column in the lower band

    assert node._box_has_trunk(depth, 0, 0, 39, 39) is True


def test_box_has_trunk_rejects_canopy_only():
    node = _make_trunk_node()
    depth = np.full((40, 40), 8.0, dtype=np.float32)  # uniform: no near column

    assert node._box_has_trunk(depth, 0, 0, 39, 39) is False
