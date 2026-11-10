"""Keep real executor state through repeated assisted measurements"""

import copy
import math
import os
import signal
import subprocess
import time
from array import array

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Point32, PolygonStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rs1_interfaces.action import MeasureTrees
from rs1_interfaces.msg import TreeLocation, PendingTreeMeasurement, TreeSamplingTarget
from rs1_interfaces.srv import LocateTree, MeasureDBH
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('occupied_maps', [False, True])
@pytest.mark.parametrize('cancel_first', [False, True])
def test_repeated_assisted_measurements(occupied_maps, cancel_first, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '177')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('assisted_sampling_test')
    broadcaster = TransformBroadcaster(node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = [node.create_publisher(OccupancyGrid, '/husky1/' + scope + '_costmap/costmap', qos)
            for scope in ('global', 'local')]
    footprint = node.create_publisher(PolygonStamped, '/husky1/local_costmap/published_footprint', 1)
    position = [100., 0.]
    map_yaw = 0.
    requests = []
    first_reply = Future()
    disagree = False

    def publish():
        world = TransformStamped()
        world.header.frame_id = 'world'
        world.header.stamp = node.get_clock().now().to_msg()
        world.child_frame_id = 'husky1_map'
        world.transform.translation.x = 100.
        world.transform.rotation.z = math.sin(map_yaw/2)
        world.transform.rotation.w = math.cos(map_yaw/2)
        robot = TransformStamped()
        robot.header.frame_id = 'husky1_map'
        robot.header.stamp = world.header.stamp
        robot.child_frame_id = 'husky1_base_link'
        dx, dy = position[0]-100., position[1]
        robot.transform.translation.x = math.cos(map_yaw)*dx + math.sin(map_yaw)*dy
        robot.transform.translation.y = -math.sin(map_yaw)*dx + math.cos(map_yaw)*dy
        # Park sideways to prove there is no face-tree heading requirement
        robot.transform.rotation.z = math.sin((1.3-map_yaw)/2)
        robot.transform.rotation.w = math.cos((1.3-map_yaw)/2)
        broadcaster.sendTransform([world, robot])
        if occupied_maps:
            grid = OccupancyGrid()
            grid.header = robot.header
            grid.info.resolution = 1.
            grid.info.width = grid.info.height = 100
            grid.info.origin.position.x = grid.info.origin.position.y = -50.
            grid.info.origin.orientation.w = 1.
            grid.data = array('b', [100])*10000
            for publisher in maps:
                publisher.publish(grid)
            shape = PolygonStamped(header=robot.header)
            shape.header.frame_id = 'husky1_base_link'
            shape.polygon.points = [Point32(x=x, y=y) for x, y in
                                    ((.6, .4), (.6, -.4), (-.6, -.4), (-.6, .4))]
            footprint.publish(shape)

    def locate(request, response):
        response.success = True
        response.centre = request.expected
        return response

    async def measure(request, response):
        requests.append(copy.deepcopy(request))
        if cancel_first and len(requests) == 1:
            await first_reply
        response.success = True
        response.dbh_m = (.68 if len(requests) == 1 else .82 if disagree else .72) if request.tree_index == 0 else .27
        return response

    node.create_service(LocateTree, '/husky1/locate_tree', locate)
    node.create_service(MeasureDBH, '/husky1/measure_dbh', measure,
                        callback_group=ReentrantCallbackGroup())
    node.create_timer(.025, publish)
    client = ActionClient(node, MeasureTrees, '/husky1/measure_trees')
    log_path = tmp_path/'husky.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([os.environ['HUSKY_EXECUTABLE'], '--ros-args',
                                    '-p', 'waypoint_wait_timeout:=1.0', '-p', 'dbh_timeout_s:=3.0'],
                                   stdout=log, stderr=log)

    def until(condition):
        deadline = time.monotonic()+12
        while not condition():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic() < deadline, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.01)

    def result(future):
        until(future.done)
        return future.result()

    def resume(goal):
        publish()
        handle = result(client.send_goal_async(goal))
        assert handle.accepted
        return result(handle.get_result_async())

    def save(goal, outcome):
        goal.survey.measurements = outcome.result.measurements
        goal.survey.pending_views = outcome.result.pending_views

    try:
        until(client.server_is_ready)
        goal = MeasureTrees.Goal(resume_from_current_pose=True)
        goal.survey.tree_map.header.frame_id = 'world'
        goal.survey.tree_map.header.stamp = node.get_clock().now().to_msg()
        goal.survey.tree_map.trees = [TreeLocation(location=Point(x=x)) for x in (102., 110., 118.)]
        invalid = copy.deepcopy(goal)
        invalid.survey.pending_views = [PendingTreeMeasurement(tree_index=3, dbh_m=.68)]
        assert not result(client.send_goal_async(invalid)).accepted

        handle = result(client.send_goal_async(goal))
        assert handle.accepted
        future = handle.get_result_async()
        if cancel_first:
            until(lambda: len(requests) == 1)
            assert result(handle.cancel_goal_async()).goals_canceling
            first_reply.set_result(None)
        first = result(future)
        assert first.status == (GoalStatus.STATUS_CANCELED if cancel_first else GoalStatus.STATUS_ABORTED)
        assert not first.result.measurements
        assert len(first.result.pending_views) == 1
        pending = copy.deepcopy(first.result.pending_views[0])
        assert pending.tree_index == 0 and pending.dbh_m == pytest.approx(.68)
        assert pending.viewpoint.x == pytest.approx(100.)
        save(goal, first)

        # Returning to the same side must not turn one view into two
        same = resume(goal)
        assert 'Second view needed' in same.result.message
        assert same.result.pending_views == [pending] and len(requests) == 1

        # Another tree can be completed without dropping the first tree's unfinished view
        position[:] = [108., 0.]
        other = resume(goal)
        assert other.status == GoalStatus.STATUS_ABORTED
        assert [m.tree_index for m in other.result.measurements] == [1]
        assert other.result.pending_views == [pending]
        save(goal, other)
        confirmed = copy.deepcopy(goal.survey.measurements)

        # Already measured trees are excluded, even when the operator parks beside one again
        count = len(requests)
        already = resume(goal)
        assert 'No unmeasured tree nearby' in already.result.message
        assert len(requests) == count and already.result.measurements == confirmed
        assert already.result.pending_views == [pending]

        position[:] = [101.8, 0.]
        close = resume(goal)
        assert 'Outside the measurement range' in close.result.message
        assert len(requests) == count and close.result.pending_views == [pending]

        # Rotate the map frame too, so the saved bearing really must use survey-frame positions
        map_yaw = .4
        position[:] = [102., -1.5]
        disagree = True
        bad = resume(goal)
        assert 'two views disagree' in bad.result.message
        assert bad.result.pending_views == [pending] and bad.result.measurements == confirmed
        disagree = False
        second = resume(goal)
        assert second.status == GoalStatus.STATUS_ABORTED
        assert not second.result.pending_views
        assert second.result.measurements[0] == confirmed[0]
        assert second.result.measurements[1].tree_index == 0
        assert second.result.measurements[1].dbh_m == pytest.approx(.70)
        save(goal, second)

        position[:] = [116.5, 0.]
        last = resume(goal)
        assert last.status == GoalStatus.STATUS_SUCCEEDED
        assert [m.tree_index for m in last.result.measurements] == [1, 0, 2]
        assert not last.result.pending_views
        assert [r.tree_index for r in requests] == [0, 1, 0, 0, 2]
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('scenario', ['map_offset', 'alternative', 'too_far', 'too_close', 'outside_search', 'neighbour_bound'])
def test_assisted_range_and_candidate_selection(scenario, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '177')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('assisted_candidate_test')
    broadcaster = TransformBroadcaster(node)
    targets, located, measured = [], [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TreeSamplingTarget, '/husky1/tree_sampling_target', targets.append, qos)

    def publish():
        world = TransformStamped()
        world.header.frame_id, world.child_frame_id = 'world', 'husky1_map'
        world.header.stamp = node.get_clock().now().to_msg()
        world.transform.translation.x = 100.
        world.transform.rotation.z, world.transform.rotation.w = math.sin(.2), math.cos(.2)
        robot = TransformStamped()
        robot.header.frame_id, robot.child_frame_id = 'husky1_map', 'husky1_base_link'
        robot.header.stamp = world.header.stamp
        robot.transform.rotation.w = 1.
        broadcaster.sendTransform([world, robot])

    def locate(request, response):
        located.append(copy.deepcopy(request))
        if scenario == 'alternative' and request.expected.x == 103.:
            response.message = 'No clear trunk from this side'
            return response
        response.success = True
        response.centre = copy.deepcopy(request.expected)
        offset = {'map_offset': -1.7, 'alternative': -.7, 'too_far': -.3,
                  'too_close': -.7, 'neighbour_bound': .5}.get(scenario, 0.)
        response.centre.x += offset
        return response

    def measure(request, response):
        measured.append(copy.deepcopy(request))
        response.success, response.dbh_m = True, .2
        return response

    node.create_service(LocateTree, '/husky1/locate_tree', locate)
    node.create_service(MeasureDBH, '/husky1/measure_dbh', measure)
    node.create_timer(.025, publish)
    client = ActionClient(node, MeasureTrees, '/husky1/measure_trees')
    log_path = tmp_path/'husky.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([os.environ['HUSKY_EXECUTABLE'], '--ros-args',
                                    '-p', 'waypoint_wait_timeout:=1.5'], stdout=log, stderr=log)

    def until(condition):
        deadline = time.monotonic()+12
        while not condition():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic() < deadline, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        until(client.server_is_ready)
        goal = MeasureTrees.Goal(resume_from_current_pose=True)
        goal.survey.mission_id = 'test-mission'
        goal.survey.tree_map.header.frame_id = 'world'
        positions = {'map_offset': [105.2], 'alternative': [103., 104.5],
                     'too_far': [104.8], 'too_close': [101.2], 'outside_search': [107.],
                     'neighbour_bound': [103., 103.6]}[scenario]
        goal.survey.tree_map.trees = [TreeLocation(location=Point(x=x)) for x in positions]
        future = client.send_goal_async(goal)
        until(future.done)
        assert future.result().accepted
        outcome = future.result().get_result_async()
        until(outcome.done)
        result = outcome.result().result
        if scenario in ('map_offset', 'alternative'):
            index = 1 if scenario == 'alternative' else 0
            assert [r.tree_index for r in measured] == [index]
            assert [r.tree_index for r in result.measurements] == [index]
            assert measured[0].tree.location.x == pytest.approx(103.8 if index else 103.5)
            assert any(t.refined and t.active and t.tree_index == index for t in targets)
            if scenario == 'map_offset':
                assert outcome.result().status == GoalStatus.STATUS_SUCCEEDED
        else:
            assert not measured
            if scenario == 'too_far':
                assert '4.50 m' in result.message and '0.50 m closer' in result.message
            elif scenario == 'too_close':
                assert '0.50 m farther away' in result.message
            elif scenario == 'outside_search':
                assert not located
                assert '5.80 m' in result.message and '1.20 m closer' in result.message
            else:
                assert len(located) == 2
                assert all(r.search_radius_m == pytest.approx(.3) for r in located)
                assert 'Could not identify trunk' in result.message
                assert '0.50 m exceeds identity limit 0.30 m' in result.message
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
