#!/usr/bin/env python3
"""Level Husky's LiDAR for its planar navigation frame, using measured IMU tilt."""
import copy
import math

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def rotation(q):
    """Return a rotation matrix, rejecting unavailable or corrupt orientations."""
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError("Invalid orientation")
    x, y, z, w = values / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def tilt_rotation(body_to_world):
    """Remove yaw: Nav2 already obtains heading from the planar EKF."""
    yaw = math.atan2(body_to_world[1, 0], body_to_world[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    world_to_heading = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return world_to_heading @ body_to_world


class LevelHuskyCloud(Node):
    def __init__(self):
        super().__init__("level_husky_cloud")
        self.base_frame = self.declare_parameter("base_frame", "husky1_base_link").value
        self.level_frame = self.declare_parameter("level_frame", "husky1_base_level").value
        self.max_imu_age = self.declare_parameter("max_imu_age", 0.08).value
        self.self_half_length = self.declare_parameter("self_half_length", 0.55).value
        self.self_half_width = self.declare_parameter("self_half_width", 0.35).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # The upstream segmentation node requests reliable delivery in cloud-only mode.
        self.publisher = self.create_publisher(PointCloud2, "points_level", 10)

        # The EKF base frame is already planar. This virtual frame describes points
        # corrected into that convention; it does not claim the physical chassis is level.
        self.broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.level_frame
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)

        self.cloud_sub = message_filters.Subscriber(
            self, PointCloud2, "scan/points", qos_profile=qos_profile_sensor_data)
        self.imu_sub = message_filters.Subscriber(
            self, Imu, "imu", qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cloud_sub, self.imu_sub], queue_size=30, slop=self.max_imu_age)
        self.sync.registerCallback(self.process)

    def process(self, cloud, imu):
        # An absent orientation must not silently produce an uncorrected scan.
        if imu.orientation_covariance[0] < 0:
            self.get_logger().warning("IMU has no orientation; skipping cloud", throttle_duration_sec=5.0)
            return
        try:
            stamp = rclpy.time.Time.from_msg(cloud.header.stamp)
            sensor = self.buffer.lookup_transform(self.base_frame, cloud.header.frame_id, stamp)
            mounting = self.buffer.lookup_transform(
                imu.header.frame_id, self.base_frame, rclpy.time.Time.from_msg(imu.header.stamp))
            body_to_world = rotation(imu.orientation) @ rotation(mounting.transform.rotation)
            level = tilt_rotation(body_to_world)

            # Preserve every field and the organised layout, changing only XYZ.
            # The sensor offset must rotate too, otherwise pitching the chassis
            # shifts the corrected obstacle positions by the LiDAR lever arm.
            # deepcopy already owns a writable byte array. Reassigning data as a
            # Python sequence would make ROS validate every byte again each scan.
            output = copy.deepcopy(cloud)
            fields = {f.name: f for f in cloud.fields}
            dtype = ">f4" if cloud.is_bigendian else "<f4"
            coordinates = []
            for name in ("x", "y", "z"):
                if fields[name].datatype != 7:
                    raise ValueError("Expected float32 XYZ fields")
                coordinates.append(np.ndarray(
                    (cloud.height, cloud.width), dtype=dtype, buffer=output.data,
                    offset=fields[name].offset, strides=(cloud.row_step, cloud.point_step)))
            xyz = np.stack(coordinates, axis=-1)
            t = sensor.transform.translation
            # No-return rays contain infinities. Keep them invalid rather than
            # allowing them to become finite obstacles during the transformation.
            with np.errstate(invalid="ignore"):
                body = xyz @ rotation(sensor.transform.rotation).T + [t.x, t.y, t.z]
                corrected = body @ level.T
            # Remove reflections from the chassis/sensor assembly before fitting
            # ground planes. Keep ground below the chassis available to segmentation.
            self_hit = ((np.abs(body[..., 0]) < self.self_half_length) &
                        (np.abs(body[..., 1]) < self.self_half_width) &
                        (body[..., 2] > -0.02) & (body[..., 2] < 0.9))
            corrected[self_hit] = np.nan
            for index, coordinate in enumerate(coordinates):
                coordinate[:] = corrected[..., index]
            output.header.frame_id = self.level_frame
            output.is_dense = False
            self.publisher.publish(output)
        except (TransformException, ValueError, KeyError) as error:
            self.get_logger().warning(f"Cannot level cloud: {error}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = LevelHuskyCloud()
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
