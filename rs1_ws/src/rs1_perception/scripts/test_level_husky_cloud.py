#!/usr/bin/env python3
"""Check the frame convention without requiring a running simulation."""
import math
from pathlib import Path
import unittest

import numpy as np
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
from level_husky_cloud import rotation, tilt_rotation
from ground_relative_scan_cloud import points_from_cloud, select_obstacles


class CloudGeometryTest(unittest.TestCase):
    def test_saved_forest_foliage_strip_is_rejected(self):
        file = Path(__file__).resolve().parents[1] / "test/data/forest_overhead_strip.npz"
        with np.load(file) as data:
            ground, point = data['ground'], data['point']
            self.assertGreater(float(data['terrain_clearance_m']), 2.5)
        np.testing.assert_array_equal(
            select_obstacles(ground, point, 0.75, 0.08, 1.0,
                             height_bound_enabled=False), point)
        self.assertEqual(len(select_obstacles(ground, point, 0.75, 0.08, 1.0)), 0)

    def test_narrow_ground_strip_rejects_only_clearly_overhead_returns(self):
        y = np.linspace(-0.7, 0.7, 8)
        ground = np.column_stack((np.full(8, 1.2), y, np.full(8, -0.42)))
        candidates = np.array([[0, 0, -0.4], [0, 0, 0.39], [0, 0, 1.5], [0, 0, 2.27]])

        # A branch well above this strip can go, but keep low and uncertain returns
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates[:3])
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0,
                             height_bound_enabled=False), candidates)

    def test_height_bound_preserves_obstacles_uphill_from_a_ground_strip(self):
        y = np.linspace(-0.7, 0.7, 8)
        ground = np.column_stack((np.zeros(8), y, 0.3 * y))
        # Actual terrain rises across the direction the scan strip cannot constrain
        actual_ground = 0.65 * 1.2
        candidates = np.array([[1.2, 0, actual_ground + h] for h in (0.1, 0.5, 0.95)])
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates)

    def test_height_bound_requires_consistent_ground_and_enough_support(self):
        y = np.linspace(-0.7, 0.7, 8)
        candidates = np.array([[1.2, 0, 4.0]])
        for heights in (np.where(y < 0, 0.0, 0.6), 0.9 * y):
            ground = np.column_stack((np.zeros(8), y, heights))
            np.testing.assert_allclose(
                select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates)
        ground = np.column_stack((np.zeros(8), y, np.zeros(8)))
        for support in (ground[:5], ground + [5, 0, 0], np.empty((0, 3))):
            np.testing.assert_allclose(
                select_obstacles(support, candidates, 0.75, 0.08, 1.0), candidates)

    def test_height_bound_respects_uncertainty_margin(self):
        y = np.linspace(-0.7, 0.7, 8)
        ground = np.column_stack((np.full(8, 1.2), y, np.zeros(8)))
        candidates = np.array([[0, 0, 2.3]])
        self.assertEqual(len(select_obstacles(ground, candidates, 0.75, 0.08, 1.0)), 0)
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0, bound_margin=0.5), candidates)

    def test_empty_ground_message_is_valid(self):
        self.assertEqual(points_from_cloud(PointCloud2()).shape, (0, 3))

    def test_ground_relative_selection_on_uphill_patch(self):
        x, y = np.meshgrid(np.linspace(4.5, 5.5, 15), np.linspace(-0.5, 0.5, 15))
        ground = np.column_stack((x.ravel(), y.ravel(), (0.3 * x).ravel()))
        candidates = np.array([[5, 0, 1.52], [5, 0, 2.0], [5, 0, 3.0], [20, 0, 4.0]])
        # Remove residual ground and overhead points. Retain the uphill obstacle
        # even though its absolute Z is 2 m, and retain an unobserved-ground point.
        selected = select_obstacles(ground, candidates, 0.75, 0.08, 1.0)
        np.testing.assert_allclose(selected, candidates[[1, 3]])

    def test_no_ground_or_collinear_ground_does_not_clear_obstacles(self):
        candidates = np.array([[0.0, 0.0, 0.02]])
        line = np.column_stack((np.linspace(-0.5, 0.5, 15), np.zeros(15), np.zeros(15)))
        for ground in [np.empty((0, 3)), line]:
            np.testing.assert_allclose(select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates)

    def test_dense_ring_does_not_hide_surrounding_ground_support(self):
        # Reproduce the bank failure: many nearby points lie on one scan ring,
        # while sparser surrounding returns describe the actual uphill surface.
        x, y = np.meshgrid(np.linspace(-0.6, 0.6, 7), np.linspace(-0.6, 0.6, 7))
        surrounding = np.column_stack((x.ravel(), y.ravel(), (0.3 * x).ravel()))
        ring_x = np.linspace(-0.15, 0.15, 101)
        ring = np.column_stack((ring_x, np.zeros_like(ring_x), 0.3 * ring_x))
        ground = np.vstack((ring, surrounding))
        candidates = np.array([[0, 0, 0.01], [0, 0, 0.12], [0, 0, 0.5]])
        # Remove the residual ground but preserve both low and taller obstacles.
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates[1:])

    def test_dense_tiny_patch_is_not_enough_to_clear_obstacles(self):
        # Many observations within one square are still only one ground sample.
        x, y = np.meshgrid(np.linspace(0.01, 0.09, 10), np.linspace(0.01, 0.09, 10))
        ground = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        candidates = np.array([[0.05, 0.05, 0.01]])
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates)

    def test_wider_ground_support_removes_overhead_but_keeps_uphill_obstacles(self):
        # Ground is visible around the gap, but nothing is within 75 cm of the tree
        angle = np.arange(12) * 2 * math.pi / 12
        x, y = 1.1 * np.cos(angle), 1.1 * np.sin(angle)
        ground = np.column_stack((x, y, 1.5 + 0.3 * x))
        candidates = np.array([
            [0, 0, 1.52], [0, 0, 1.65], [0, 0, 2.45], [0, 0, 2.7], [8, 0, 5.0]])

        # The original radius cannot classify anything in this gap
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0, 0.75), candidates)
        # The fallback keeps low obstacles, low branches and the unobserved area
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates[[1, 2, 4]])

    def test_wider_fit_still_requires_all_existing_quality_checks(self):
        angle = np.arange(12) * 2 * math.pi / 12
        x, y = 1.1 * np.cos(angle), 1.1 * np.sin(angle)
        ring = np.column_stack((x, y, np.zeros(12)))
        line_x = np.r_[np.linspace(-1.4, -0.8, 6), np.linspace(0.8, 1.4, 6)]
        grounds = {
            'too_few_samples': np.vstack((ring[:5], [[8, 0, 0], [8, 1, 0]])),
            'collinear': np.column_stack((line_x, np.zeros((12, 2)))),
            'poor_plane_fit': np.column_stack((x, y, np.tile([-0.2, 0.2], 6))),
            'too_steep': np.column_stack((x, y, 0.8 * x)),
            'outside_fallback_radius': ring * 2,
        }
        candidates = np.array([[0, 0, 0.02], [0, 0, 0.4], [0, 0, 2.0]])
        for reason, ground in grounds.items():
            with self.subTest(reason=reason):
                np.testing.assert_allclose(
                    select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates)

    def test_good_local_fit_is_not_replaced_by_a_different_wider_patch(self):
        angle = np.arange(6) * 2 * math.pi / 6
        inner = np.column_stack((0.5 * np.cos(angle), 0.5 * np.sin(angle), np.zeros(6)))
        outer = np.column_stack((1.1 * np.cos(angle), 1.1 * np.sin(angle), np.full(6, 1.2)))
        ground = np.vstack((inner, outer))
        candidates = np.array([[0, 0, 0.4], [0, 0, 1.4]])

        # A broad fit mixes two ground levels and fails, but the local fit is good
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 1.5, 0.08, 1.0), candidates)
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates[:1])

    def test_local_fallback_and_unknown_ground_can_share_one_scan(self):
        angle = np.arange(12) * 2 * math.pi / 12
        local = np.column_stack((0.5 * np.cos(angle), 0.5 * np.sin(angle), np.zeros(12)))
        distant = np.column_stack((5 + 1.1 * np.cos(angle), 1.1 * np.sin(angle), np.full(12, 1.5)))
        ground = np.vstack((local, distant))
        candidates = np.array([
            [0, 0, 1.3], [5, 0, 1.8], [12, 0, 4], [0, 0, 0.4], [5, 0, 2.8]])
        np.testing.assert_allclose(
            select_obstacles(ground, candidates, 0.75, 0.08, 1.0), candidates[[1, 2, 3]])

    def test_pitch_restores_horizontal_ground_and_sensor_offset(self):
        # Construct a pitched sensor observation of points on horizontal ground.
        # Correcting both its coordinates and its mounting offset must recover them.
        angle = math.radians(15)
        q = Quaternion(y=math.sin(angle / 2), w=math.cos(angle / 2))
        body_to_world = rotation(q)
        ground = np.array([[2, 1, -0.13228], [8, -2, -0.13228]])
        mount = np.array([0, 0, 0.68])
        sensor_points = ground @ body_to_world - mount
        corrected = (sensor_points + mount) @ tilt_rotation(body_to_world).T
        np.testing.assert_allclose(corrected, ground, atol=1e-10)

    def test_yaw_is_not_applied_twice(self):
        yaw = 1.2
        q = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        np.testing.assert_allclose(tilt_rotation(rotation(q)), np.eye(3), atol=1e-10)

    def test_roll_with_yaw_preserves_robot_heading(self):
        yaw, roll = 1.2, -0.25
        heading = rotation(Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
        tilt = rotation(Quaternion(x=math.sin(roll / 2), w=math.cos(roll / 2)))
        np.testing.assert_allclose(tilt_rotation(heading @ tilt), tilt, atol=1e-10)

    def test_invalid_orientation_is_rejected(self):
        for q in [Quaternion(w=0.0), Quaternion(w=float("nan"))]:
            with self.assertRaises(ValueError):
                rotation(q)


if __name__ == "__main__":
    unittest.main()
