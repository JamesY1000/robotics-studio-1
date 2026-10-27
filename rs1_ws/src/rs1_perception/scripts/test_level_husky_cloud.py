#!/usr/bin/env python3
"""Check the frame convention without requiring a running simulation."""
import math
import unittest

import numpy as np
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
from level_husky_cloud import rotation, tilt_rotation
from ground_relative_scan_cloud import points_from_cloud, select_obstacles


class CloudGeometryTest(unittest.TestCase):
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
