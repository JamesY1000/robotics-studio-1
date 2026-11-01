#!/usr/bin/env python3
"""Convert Husky's body-frame gyro measurements for its planar EKF."""
import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def measurement_covariance(values, fallback_variance):
    """Use a nonzero uncertainty when the IMU reports an unknown covariance."""
    if not all(math.isfinite(value) for value in values):
        raise ValueError("IMU covariance is not finite")
    if any(values[index] < 0.0 for index in (0, 4, 8)):
        raise ValueError("IMU measurement is unavailable or has invalid covariance")
    if not any(values):
        # sensor_msgs/Imu defines an all-zero covariance as unknown, not exact.
        return [fallback_variance, 0.0, 0.0,
                0.0, fallback_variance, 0.0,
                0.0, 0.0, fallback_variance]
    return values


def heading_measurement(imu, gyro_variance, orientation_variance):
    """Calculate horizontal heading rate and its first-order uncertainty.

    Husky's imu_link has the same axis directions as base_link in its URDF.
    This calculation assumes that alignment; a rotated IMU mounting would need
    its orientation and gyro measurements transformed into the body frame first.
    """
    q = imu.orientation
    values = [q.x, q.y, q.z, q.w]
    rates = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
    if not all(math.isfinite(value) for value in values + rates):
        raise ValueError("IMU orientation or angular velocity is not finite")
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("IMU orientation quaternion is invalid")
    x, y, z, w = [value / norm for value in values]

    roll = math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w*y - z*x))))
    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)
    cos_pitch = math.cos(pitch)
    if cos_pitch < 0.1:
        # Heading becomes ill-defined near vertical. Do not feed an amplified
        # or undefined rate into the EKF if the robot tips over.
        raise ValueError("Husky is too close to vertical for planar heading")

    # Raw gyro Z measures rotation about the tilted chassis Z axis. The EKF
    # instead needs the change in heading on the horizontal map (radians/sec).
    gyro_weights = [0.0, sin_roll / cos_pitch, cos_roll / cos_pitch]
    heading_rate = sum(weight * rate for weight, rate in zip(gyro_weights, rates))

    # Propagate gyro uncertainty through the same projection: variance = J C J^T.
    gyro_covariance = measurement_covariance(imu.angular_velocity_covariance, gyro_variance)
    variance = sum(gyro_weights[row] * gyro_covariance[3*row + column] * gyro_weights[column]
                   for row in range(3) for column in range(3))
    if variance < 0.0:
        raise ValueError("IMU gyro covariance produces a negative variance")

    # Tilt uncertainty also affects the projected rate. Use an upper bound on
    # orientation variance so this does not depend on the covariance's choice
    # of rotation axes. The largest absolute row sum bounds its eigenvalues.
    orientation_covariance = measurement_covariance(
        imu.orientation_covariance, orientation_variance)
    rotation_variance = max(
        sum(abs(orientation_covariance[3*row + column]) for column in range(3))
        for row in range(3))
    roll_derivative = (cos_roll * rates[1] - sin_roll * rates[2]) / cos_pitch
    pitch_derivative = heading_rate * math.tan(pitch)
    # Convert roll/pitch derivatives to sensitivity to a small body rotation.
    tilt_weights = [roll_derivative,
                    roll_derivative * sin_roll * math.tan(pitch) + pitch_derivative * cos_roll,
                    roll_derivative * cos_roll * math.tan(pitch) - pitch_derivative * sin_roll]
    variance += rotation_variance * sum(weight * weight for weight in tilt_weights)
    if not math.isfinite(heading_rate) or not math.isfinite(variance):
        raise ValueError("Projected IMU measurement is not finite")
    return heading_rate, max(variance, 1e-9)


class HuskyHeadingRate(Node):
    def __init__(self):
        super().__init__("husky_heading_rate")
        self.imu_frame = self.declare_parameter("imu_frame", "husky1_imu_link").value
        self.base_frame = self.declare_parameter("base_frame", "husky1_base_link").value
        # Gazebo's configured gyro noise is 2e-4 rad/s, giving variance 4e-8.
        # Orientation has no supplied covariance, so assume one degree of
        # uncertainty as a conservative fallback. These defaults need revisiting
        # when replacing the simulated IMU with a real sensor.
        self.gyro_variance = self.declare_parameter("gyro_variance", 4e-8).value
        self.orientation_variance = self.declare_parameter(
            "orientation_variance", math.radians(1.0)**2).value
        for variance in (self.gyro_variance, self.orientation_variance):
            if not math.isfinite(variance) or variance <= 0.0:
                raise ValueError("Fallback IMU variances must be finite and positive")
        self.publisher = self.create_publisher(TwistWithCovarianceStamped, "heading_rate", 10)
        self.subscription = self.create_subscription(
            Imu, "imu", self.process, qos_profile_sensor_data)

    def process(self, imu):
        try:
            if imu.header.frame_id != self.imu_frame:
                raise ValueError("Unexpected IMU frame: " + imu.header.frame_id)
            rate, variance = heading_measurement(
                imu, self.gyro_variance, self.orientation_variance)
        except ValueError as error:
            self.get_logger().warning(
                "Cannot calculate heading rate: " + str(error), throttle_duration_sec=5.0)
            return

        # The existing EKF base frame is planar even when the physical chassis
        # tilts. Only angular.z in this message is a measurement; the EKF config
        # deliberately ignores all other components. Keep the sensor timestamp
        # so callback scheduling does not change when the measurement occurred.
        output = TwistWithCovarianceStamped()
        output.header.stamp = imu.header.stamp
        output.header.frame_id = self.base_frame
        output.twist.twist.angular.z = rate
        output.twist.covariance[35] = variance
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = HuskyHeadingRate()
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
