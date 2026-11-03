"""Exercise the installed service with two nearby trunks and a transformed survey map."""

import math
import json
import signal
import subprocess
import time
import sys
from pathlib import Path

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rs1_interfaces.srv import LocateTree, MeasureDBH
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dbh_3d_detector import DetectionParams, trunk_widths


def scan(trunks, stem_height=math.inf, stem_base=0.0):
    # Cast Husky's 17 rings against level ground and the visible side of each trunk
    az, el = np.meshgrid(np.deg2rad(np.arange(1440) * 0.25), np.deg2rad(np.linspace(-30, 30, 17)))
    rays = np.column_stack(((np.cos(az) * np.cos(el)).ravel(),
                            (np.sin(az) * np.cos(el)).ravel(), np.sin(el).ravel()))
    x, y, z = rays.T
    with np.errstate(divide="ignore"):
        ranges = np.where(z < 0, -0.81 / z, np.inf)
    a = x*x + y*y
    for cx, cy, diameter in trunks:
        b = x*cx + y*cy
        disc = b*b - a*(cx*cx + cy*cy - (diameter/2)**2)
        hit = (b - np.sqrt(np.maximum(disc, 0))) / a
        height = 0.81 + hit * z
        visible = (disc >= 0) & (hit > 0) & (height < stem_height) & (height >= stem_base)
        ranges = np.minimum(ranges, np.where(visible, hit, np.inf))
    keep = (ranges < 40) & (ranges > 0.2)
    points = (ranges[keep, None] * rays[keep]).astype(np.float32)
    return create_cloud_xyz32(Header(frame_id="laser"), points)


@pytest.fixture
def service(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "177")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    node = rclpy.create_node("measure_dbh_test")
    client = node.create_client(MeasureDBH, "/husky1/measure_dbh")
    pub = node.create_publisher(PointCloud2, "/husky1/scan/points", qos_profile_sensor_data)
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "world", "laser"
    translation = tf.transform.translation
    translation.x, translation.y, translation.z = 10., -3., 0.81
    tf.transform.rotation.z = tf.transform.rotation.w = math.sqrt(0.5)
    broadcaster.sendTransform(tf)
    binary = Path(get_package_prefix("rs1_perception")) / "lib/rs1_perception/measure_dbh.py"
    log = tmp_path / "server.log"
    with log.open("w") as stream:
        process = subprocess.Popen([str(binary), "--ros-args", "-p", "measurement_timeout_s:=2.5",
                                    "-p", f"diagnostics_dir:={tmp_path / 'captures'}"],
                                   stdout=stream, stderr=subprocess.STDOUT)

    def until(condition, timeout=6):
        deadline = time.monotonic() + timeout
        while not condition():
            assert process.poll() is None, log.read_text()
            assert time.monotonic() < deadline, log.read_text()
            rclpy.spin_once(node, timeout_sec=0.02)

    def request(x, y, index=1, *, max_offset=0.0):
        req = MeasureDBH.Request()
        req.map_header.frame_id = "world"
        req.tree_index = index
        req.max_center_offset_m = max_offset
        req.tree.location.x, req.tree.location.y = 10. - y, -3. + x
        return client.call_async(req)

    def publish(cloud, stamp=None):
        cloud.header.stamp = stamp or node.get_clock().now().to_msg()
        pub.publish(cloud)

    try:
        until(lambda: client.service_is_ready() and pub.get_subscription_count() == 1, 10)
        yield node, request, publish, until
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('bearing', [0., math.pi/2, math.pi, -math.pi/2])
def test_requested_tree_with_neighbour(service, bearing):
    node, request, publish, until = service
    # The thicker tree is closer to the scanner and both trees are inside the point crop
    trunks = [(math.cos(bearing)*x-math.sin(bearing)*y,
               math.sin(bearing)*x+math.cos(bearing)*y, diameter)
              for x, y, diameter in [(1.50, 0.05, 0.20), (1.29, -0.63, 0.42)]]
    cloud = scan(trunks)
    for index, (x, y, expected) in enumerate(trunks):
        future = request(x, y, index)
        timer = node.create_timer(0.10, lambda: publish(cloud))
        until(future.done)
        node.destroy_timer(timer)
        result = future.result()
        assert result.success, result.message
        assert result.dbh_m == pytest.approx(expected, abs=0.015)


def test_only_distinct_post_request_scans_count(service):
    node, request, publish, until = service
    # Delayed scans and copies of the first fresh scan must not fill the five-scan window
    old = scan([(1.5, 0., 0.40)])
    fresh = scan([(1.5, 0., 0.20)])
    old_stamp = node.get_clock().now().to_msg()
    publish(old, old_stamp)
    future = request(1.5, 0.)
    time.sleep(0.15)
    one_stamp = node.get_clock().now().to_msg()
    for _ in range(6):
        publish(old, old_stamp)
        time.sleep(0.025)
        publish(fresh, one_stamp)
        rclpy.spin_once(node, timeout_sec=0.08)
    assert not future.done()
    timer = node.create_timer(0.10, lambda: publish(fresh))
    until(future.done)
    node.destroy_timer(timer)
    assert future.result().success, future.result().message
    assert future.result().dbh_m == pytest.approx(0.20, abs=0.015)


@pytest.mark.parametrize("trunks,reason", [
    ([(1.5, -0.29, 0.16), (1.5, 0.29, 0.22)], "Ambiguous"),
    ([(1.35, 0.65, 0.42)], "No matching trunk"),
])
def test_cannot_identify_target(service, trunks, reason, tmp_path):
    node, request, publish, until = service
    cloud = scan(trunks)
    future = request(1.5, 0.)
    timer = node.create_timer(0.10, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    assert not future.result().success
    assert reason in future.result().message
    capture = next((tmp_path / 'captures').glob('*_dbh'))
    metadata = json.loads((capture / 'request.json').read_text())
    assert not metadata['success']
    assert metadata['frame'] == 'world'
    scans = list(capture.glob('scan_*.npz'))
    assert 1 <= len(scans) <= 8
    recorded = np.load(scans[-1])
    details = json.loads(str(recorded['details']))
    assert details['cloud_frame'] == 'laser'
    assert details['target_scanner'][:2] == pytest.approx([1.5, 0.])
    assert recorded['cloud_cdr'].size > 0
    assert recorded['xyz'].shape[1] == 3


@pytest.mark.parametrize("trunks, reason", [
    ([(1.5, -0.17, 0.22), (1.5, 0.17, 0.22)], "centre moved"),
    ([(1.5, 0., 0.19), (1.5, 0., 0.31)], "Diameter varied"),
])
def test_inconsistent_scans_do_not_confirm_a_tree(service, trunks, reason):
    node, request, publish, until = service
    clouds = [scan([tree]) for tree in trunks]
    future = request(1.5, 0.)
    frame = 0

    def alternate_views():
        nonlocal frame
        publish(clouds[frame % 2])
        frame += 1

    timer = node.create_timer(0.15, alternate_views)
    until(future.done)
    node.destroy_timer(timer)
    assert not future.result().success
    assert reason in future.result().message


@pytest.mark.parametrize("neighbour", [False, True])
def test_refine_map_centre_before_standoff(service, neighbour):
    node, _, publish, until = service
    client = node.create_client(LocateTree, "/husky1/locate_tree")
    until(client.service_is_ready)
    trunks = [(2.9, 0.55, 0.25)]
    if neighbour:
        trunks.append((2.8, -0.55, 0.30))
    # Only the rings below the branches are visible from this staging pose
    cloud = scan(trunks, stem_height=0.70)
    req = LocateTree.Request()
    req.map_header.frame_id = "world"
    req.expected.x, req.expected.y = 10., -0.2
    req.search_radius_m = 0.85
    future = client.call_async(req)
    timer = node.create_timer(0.10, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    result = future.result()
    if neighbour:
        assert not result.success and "Ambiguous" in result.message
    else:
        assert result.success, result.message
        assert result.centre.x == pytest.approx(9.45, abs=0.025)
        assert result.centre.y == pytest.approx(-0.1, abs=0.025)


def test_close_neighbour_cannot_replace_missing_target(service):
    node, request, publish, until = service
    cloud = scan([(1.5, 0.24, 0.2)])
    future = request(1.5, 0., max_offset=0.12)
    timer = node.create_timer(0.10, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    assert not future.result().success
    assert "No matching trunk" in future.result().message


def test_refinement_rejects_floating_branch(service):
    node, _, publish, until = service
    client = node.create_client(LocateTree, "/husky1/locate_tree")
    until(client.service_is_ready)
    # A branch can produce two consistent rings without being a standing trunk
    cloud = scan([(2.8, 0., 0.4)], stem_base=0.85, stem_height=1.3)
    req = LocateTree.Request()
    req.map_header.frame_id = "world"
    req.expected.x, req.expected.y = 10., -0.2
    req.search_radius_m = 0.85
    future = client.call_async(req)
    timer = node.create_timer(0.10, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    assert not future.result().success


def test_dbh_rejects_floating_branch(service):
    node, request, publish, until = service
    cloud = scan([(1.5, 0., 0.4)], stem_base=0.9, stem_height=1.6)
    future = request(1.5, 0.)
    timer = node.create_timer(0.10, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    assert not future.result().success


def test_recorded_forest_foliage_is_not_a_trunk():
    capture = np.load(Path(__file__).parent / 'data/forest_false_trunk.npz')
    details = {}
    stems = trunk_widths(capture['xyz'], DetectionParams(), capture['target'],
                         float(capture['tolerance']), locating=True, diagnostics=details)
    assert stems == []
    # The old locator accepted this repeatable foliage patch as an 88 cm trunk
    assert any(min(t['heights']) > 1.0 and len(t['heights']) == 2 for t in details['tracks'])


@pytest.mark.parametrize('distance', [1.5, 2.0, 2.45, 3.3, 3.5, 3.95])
def test_wider_range_keeps_target_identity(service, distance):
    node, request, publish, until = service
    cloud = scan([(distance, 0., .20), (distance-.15, -.65, .36)])
    future = request(distance, 0.)
    timer = node.create_timer(.1, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    assert future.result().success, future.result().message
    assert future.result().dbh_m == pytest.approx(.20, abs=.015)


def test_wider_range_still_rejects_distant_target(service):
    node, request, publish, until = service
    cloud = scan([(4.5, 0., .2)])
    future = request(4.5, 0.)
    timer = node.create_timer(.1, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    assert not future.result().success
    assert 'stand-off' in future.result().message
    assert '4.50 m' in future.result().message
    assert '0.50 m closer' in future.result().message


def test_lower_stem_does_not_count_as_dbh_at_wider_range(service):
    node, request, publish, until = service
    cloud = scan([(3.3, 0., .2)], stem_height=.9)
    future = request(3.3, 0.)
    timer = node.create_timer(.1, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    result = future.result()
    assert not result.success
    assert 'breast height' in result.message
    assert 'move around the tree' in result.message


def test_locator_includes_map_uncertainty_at_distance_limit(service):
    node, _, publish, until = service
    client = node.create_client(LocateTree, '/husky1/locate_tree')
    until(client.service_is_ready)
    cloud = scan([(3.8, 0., .2)])
    req = LocateTree.Request()
    req.map_header.frame_id = 'world'
    req.expected.x, req.expected.y = 10., 2.2
    req.search_radius_m = 1.8
    future = client.call_async(req)
    timer = node.create_timer(.1, lambda: publish(cloud))
    until(future.done)
    node.destroy_timer(timer)
    assert future.result().success, future.result().message
    assert future.result().centre.y == pytest.approx(.8, abs=.03)
