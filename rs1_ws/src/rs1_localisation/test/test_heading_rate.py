"""Check the projection against 3D rotations and the ROS message contract."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Imu

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/husky_heading_rate.py"
spec = importlib.util.spec_from_file_location("husky_heading_rate", SCRIPT)
heading = importlib.util.module_from_spec(spec)
spec.loader.exec_module(heading)


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u,
            w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s)


def quaternion(roll, pitch, yaw):
    qx = (math.sin(roll/2), 0.0, 0.0, math.cos(roll/2))
    qy = (0.0, math.sin(pitch/2), 0.0, math.cos(pitch/2))
    qz = (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))
    return multiply(multiply(qz, qy), qx)


def message(q, rates):
    imu = Imu()
    imu.header.frame_id = "husky1_imu_link"
    imu.header.stamp.sec = 12
    imu.header.stamp.nanosec = 345
    imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = rates
    return imu


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


@pytest.mark.parametrize("attitude,rates", [
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.5)),
    ((0.2, -0.25, 1.2), (0.4, -0.3, 0.5)),
    ((-0.3, 0.2, -2.0), (-0.2, 0.4, -0.6)),
])
def test_rate_matches_heading_change_from_a_3d_rotation(attitude, rates):
    # Advance the complete orientation by a small body-frame rotation. This
    # checks the conversion independently of its roll/pitch projection formula.
    q = quaternion(*attitude)
    dt = 1e-7
    speed = math.sqrt(sum(value*value for value in rates))
    increment = tuple(value/speed * math.sin(speed*dt/2) for value in rates)
    next_q = multiply(q, (*increment, math.cos(speed*dt/2)))
    expected = math.atan2(math.sin(yaw(next_q)-yaw(q)), math.cos(yaw(next_q)-yaw(q))) / dt
    actual, variance = heading.heading_measurement(message(q, rates), 4e-8, 0.0003)
    assert actual == pytest.approx(expected, abs=1e-6)
    assert math.isfinite(variance) and variance > 0.0


def test_uncertainty_matches_small_independent_sensor_perturbations():
    q = quaternion(0.2, -0.3, 1.0)
    rates = (0.2, 0.4, -0.5)
    epsilon = 1e-6
    actual, variance = heading.heading_measurement(message(q, rates), 4e-8, 0.0003)
    expected_variance = 0.0
    for axis in range(3):
        perturbed_rates = list(rates)
        perturbed_rates[axis] += epsilon
        value, _ = heading.heading_measurement(message(q, perturbed_rates), 4e-8, 0.0003)
        expected_variance += ((value-actual)/epsilon)**2 * 4e-8
        increment = [0.0, 0.0, 0.0, math.cos(epsilon/2)]
        increment[axis] = math.sin(epsilon/2)
        value, _ = heading.heading_measurement(
            message(multiply(q, increment), rates), 4e-8, 0.0003)
        expected_variance += ((value-actual)/epsilon)**2 * 0.0003
    assert variance == pytest.approx(expected_variance, rel=1e-5)


@pytest.mark.parametrize(
    "fault", ["no_orientation", "no_gyro", "zero_quaternion", "nan", "vertical"])
def test_unusable_measurements_are_rejected(fault):
    imu = message(quaternion(0.0, 0.0, 0.0), (0.0, 0.0, 0.5))
    if fault == "no_orientation":
        imu.orientation_covariance[0] = -1.0
    elif fault == "no_gyro":
        imu.angular_velocity_covariance[0] = -1.0
    elif fault == "zero_quaternion":
        imu.orientation.w = 0.0
    elif fault == "nan":
        imu.angular_velocity.y = float("nan")
    else:
        imu = message(quaternion(0.0, math.pi/2, 0.0), (0.0, 0.0, 0.5))
    with pytest.raises(ValueError):
        heading.heading_measurement(imu, 4e-8, 0.0003)


def test_callback_preserves_timestamp_and_keeps_raw_imu_unchanged():
    sent = []
    warnings = []
    node = SimpleNamespace(
        imu_frame="husky1_imu_link", base_frame="husky1_base_link",
        gyro_variance=4e-8, orientation_variance=0.0003,
        publisher=SimpleNamespace(publish=sent.append),
        get_logger=lambda: SimpleNamespace(warning=lambda text, **kwargs: warnings.append(text)))
    imu = message(quaternion(0.2, -0.3, 0.0), (0.0, 0.2, 0.5))
    heading.HuskyHeadingRate.process(node, imu)
    assert len(sent) == 1
    assert sent[0].header.stamp == imu.header.stamp
    assert sent[0].header.frame_id == "husky1_base_link"
    assert sent[0].twist.covariance[35] > 0.0
    assert imu.header.frame_id == "husky1_imu_link"
    assert imu.angular_velocity.z == 0.5
    imu.header.frame_id = "unexpected_frame"
    heading.HuskyHeadingRate.process(node, imu)
    assert len(sent) == 1
    assert warnings
