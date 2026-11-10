"""Run the Husky survey against controlled Nav2 goals and DBH readings"""

import math
import copy
from array import array
import os
import signal
import subprocess
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rs1_interfaces.action import MeasureTrees
from rs1_interfaces.msg import TreeLocation, TreeMeasurement
from rs1_interfaces.srv import LocateTree, MeasureDBH
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('scenario', [
    'survey', 'resume', 'blocked_side', 'turn', 'nav_abort', 'cancel_nav',
    'bad_reading', 'no_service', 'empty', 'invalid_resume', 'cancel_read', 'slow_read',
    'blocked_tree', 'refined', 'broad', 'retry_read', 'stuck', 'disagree',
    'plan_abort', 'path_collision', 'stall_recover', 'backup_blocked',
    'cancel_plan', 'cancel_backup', 'bad_path_end',
    'partial_backup', 'wider_after_paths', 'footprint_frame',
    'hidden_standoff',
    'unknown_space', 'unknown_with_obstacle', 'retry_budgets',
    'farther_pose', 'diverse_plans', 'shorter_path', 'disagree_15_percent', 'blocked_departure', 'goal_tolerance',
    'rear_view', 'side_view', 'coasting', 'occupied_arrival', 'closer_view',
    'new_retreat_position', 'same_retreat_position',
    'stale_arrival', 'unobserved_arrival',
    'current_pose', 'current_pose_resume', 'current_pose_bad_view', 'manual_reposition', 'wide_current_pose',
])
def test_tree_survey(scenario, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '176')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('husky_survey_test')
    broadcaster = TransformBroadcaster(node)
    grid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub = node.create_publisher(OccupancyGrid, '/husky1/global_costmap/costmap', grid_qos)
    local_pub = node.create_publisher(OccupancyGrid, '/husky1/local_costmap/costmap', grid_qos)
    footprint_pub = node.create_publisher(PolygonStamped, '/husky1/local_costmap/published_footprint', 1)
    robot_x, robot_y, yaw = 0., 0., 0.
    events = []
    requests = []
    bearings = []
    nav_goals = []
    plan_goals = []
    backup_goals = []
    nav_wait = Future()
    plan_wait = Future()
    backup_wait = Future()
    read_wait = Future()
    read_finished = False
    view_open = False
    local_blocked = False
    scan_poses = []
    arrived_at = None

    def publish():
        nonlocal robot_x
        if scenario == 'coasting' and arrived_at and time.monotonic()-arrived_at < 0.6:
            robot_x += 0.01
        stamp = node.get_clock().now().to_msg()
        pose = TransformStamped()
        pose.header.frame_id = 'husky1_map'
        pose.header.stamp = stamp
        pose.child_frame_id = 'husky1_base_link'
        pose.transform.translation.x = robot_x
        pose.transform.translation.y = robot_y
        pose.transform.rotation.z = math.sin(yaw / 2)
        pose.transform.rotation.w = math.cos(yaw / 2)
        world = TransformStamped()
        world.header.frame_id = 'world'
        world.header.stamp = stamp
        world.child_frame_id = 'husky1_map'
        world.transform.translation.x = 100.
        world.transform.rotation.w = 1.
        broadcaster.sendTransform([world, pose])
        grid = OccupancyGrid()
        grid.header = copy.deepcopy(pose.header)
        grid.info.width = grid.info.height = 160
        grid.info.resolution = .25
        grid.info.origin.position.x = grid.info.origin.position.y = -20.
        grid.info.origin.orientation.w = 1.
        grid.data = array('b', [100 if scenario == 'blocked_tree' else 0]) * 25600
        if scenario in ('unknown_space', 'unknown_with_obstacle'):
            grid.data = array('b', [-1]) * 25600
            # The route may be unknown, but the live scanner must observe the parked footprint
            ix, iy = int((robot_x+20)/.25), int((robot_y+20)/.25)
            for y in range(iy-4, iy+5):
                for x in range(ix-4, ix+5):
                    grid.data[y*160+x] = 0
        if scenario in ('blocked_side', 'unknown_with_obstacle'):
            # The goal's centre is clear but its front corner would hit this cell
            grid.data[81 * 160 + 83] = 100
        if scenario == 'path_collision':
            grid.data[88 * 160 + 88] = 100
        if scenario == 'blocked_departure' and not view_open:
            # Clear while parked, but turning toward the route swings a corner into this cell
            grid.data[83 * 160 + 80] = 100
        if scenario in ('hidden_standoff', 'farther_pose') and not view_open:
            for iy in range(74, 86):
                for ix in range(82, 94):
                    if math.hypot(-20+(ix+.5)*.25-2, -20+(iy+.5)*.25) < 1.15:
                        grid.data[iy*160+ix] = 100
        if scenario == 'stale_arrival' and arrived_at:
            grid.header.stamp.sec -= 5
        if scenario == 'unobserved_arrival' and arrived_at:
            grid.data = array('b', [-1]) * 25600
        grid_pub.publish(grid)
        if local_blocked:
            # A failed drive can be blocked locally even when the global map still looks clear
            ix = int((robot_x+20)/.25)
            iy = int((robot_y+20)/.25)
            grid.data[iy*160+ix] = 100
        elif not nav_goals and scenario not in ('current_pose', 'current_pose_resume', 'current_pose_bad_view', 'wide_current_pose'):
            # These cases exercise navigation, so the initial parked footprint hasn't been observed yet
            ix, iy = int((robot_x+20)/.25), int((robot_y+20)/.25)
            grid.data[iy*160+ix] = -1
        local_pub.publish(grid)
        footprint = PolygonStamped()
        footprint.header.stamp = copy.deepcopy(stamp)
        footprint.header.frame_id = 'husky1_base_link'
        footprint.polygon.points = [Point32(x=x, y=y) for x, y in
                                    [(.6, .5), (.6, -.5), (-.6, -.5), (-.6, .5)]]
        if scenario == 'footprint_frame':
            footprint.header.frame_id = 'world'
            for p in footprint.polygon.points:
                x, y = p.x, p.y
                p.x = 100. + robot_x + math.cos(yaw)*x - math.sin(yaw)*y
                p.y = robot_y + math.sin(yaw)*x + math.cos(yaw)*y
        footprint_pub.publish(footprint)

    async def plan(handle):
        plan_goals.append(handle)
        if scenario == 'cancel_plan':
            await plan_wait
            handle.canceled()
            return ComputePathToPose.Result()
        if (scenario == 'plan_abort' and len(plan_goals) == 1) or (scenario == 'wider_after_paths' and not nav_goals and len(plan_goals) <= 24):
            handle.abort()
            return ComputePathToPose.Result()
        if scenario in ('diverse_plans', 'goal_tolerance'):
            p = handle.request.goal.pose.position
            if math.hypot(p.x-2., p.y) < 2.1:
                handle.abort()
                return ComputePathToPose.Result()
        if scenario == 'closer_view' and requests:
            p = handle.request.goal.pose.position
            if math.hypot(p.x-2., p.y) >= 1.5-1e-6:
                handle.abort()
                return ComputePathToPose.Result()
        start = PoseStamped()
        start.header = handle.request.goal.header
        start.pose.position.x, start.pose.position.y = robot_x, robot_y
        start.pose.orientation.w = 1.
        path = Path(header=start.header, poses=[start, copy.deepcopy(handle.request.goal)])
        if scenario == 'blocked_departure' and not view_open:
            turn = PoseStamped(header=start.header)
            turn.pose.position.x = turn.pose.position.y = 1.
            turn.pose.orientation.w = 1.
            path.poses.insert(1, turn)
        if scenario == 'hidden_standoff' and not view_open:
            target = handle.request.goal.pose.position
            first = math.atan2(robot_y, robot_x-2.)
            angle = math.remainder(math.atan2(target.y, target.x-2.)-first, 2*math.pi)
            arc = []
            for step in range(9):
                pose = PoseStamped(header=start.header)
                pose.pose.position.x = 2.+2.8*math.cos(first+angle*step/8)
                pose.pose.position.y = 2.8*math.sin(first+angle*step/8)
                pose.pose.orientation.w = 1.
                arc.append(pose)
            path.poses = [start, *arc, handle.request.goal]
        if scenario == 'shorter_path' and len(plan_goals) == 1:
            detour = PoseStamped(header=start.header)
            detour.pose.position.x, detour.pose.position.y = -5., 5.
            detour.pose.orientation.w = 1.
            path.poses.insert(1, detour)
        if scenario == 'path_collision' and len(plan_goals) == 1:
            middle = PoseStamped(header=start.header)
            middle.pose.position.x = middle.pose.position.y = 2.1
            middle.pose.orientation.w = 1.
            path.poses.insert(1, middle)
        if scenario == 'bad_path_end' and len(plan_goals) == 1:
            path.poses[-1].pose.position.x = -2.
        handle.succeed()
        return ComputePathToPose.Result(path=path)

    async def backup(handle):
        nonlocal robot_x, robot_y, local_blocked, view_open
        backup_goals.append(handle)
        if scenario == 'cancel_backup':
            await backup_wait
            handle.canceled()
        elif scenario in ('blocked_tree', 'backup_blocked', 'stuck', 'occupied_arrival', 'unobserved_arrival'):
            handle.abort()
        else:
            distance = .3 if scenario == 'partial_backup' else .5
            robot_x -= distance * math.cos(yaw)
            robot_y -= distance * math.sin(yaw)
            local_blocked = False
            if scenario == 'blocked_departure':
                view_open = True
            publish()
            if scenario == 'partial_backup':
                handle.abort()
            else:
                handle.succeed()
        return BackUp.Result()

    async def navigate(handle):
        nonlocal robot_x, robot_y, yaw, view_open, local_blocked, arrived_at
        events.append('drive')
        pose = handle.request.pose
        nav_goals.append(handle)
        if scenario == 'cancel_nav' or (scenario in ('stall_recover', 'manual_reposition') and len(nav_goals) == 1):
            local_blocked = scenario == 'stall_recover'
            publish()
            await nav_wait
            handle.canceled()
        elif (scenario in ('stuck', 'backup_blocked', 'cancel_backup') or
              (scenario in ('nav_abort', 'partial_backup') and len(nav_goals) == 1) or
              (scenario in ('retry_budgets', 'new_retreat_position', 'same_retreat_position') and len(nav_goals) <= 2)):
            local_blocked = scenario != 'nav_abort'
            if scenario == 'retry_budgets':
                local_blocked = False
                # Abort while still outside DBH range so these really are two navigation attempts
                robot_x = -1.
            if scenario == 'new_retreat_position' and len(nav_goals) == 2:
                robot_x = -2.
            publish()
            handle.abort()
        else:
            robot_x, robot_y = pose.pose.position.x, pose.pose.position.y
            if scenario == 'goal_tolerance':
                # Nav2 is allowed to finish 30 cm short of the requested pose
                distance = math.hypot(robot_x-2., robot_y)
                robot_x += .3*(robot_x-2.)/distance
                robot_y += .3*robot_y/distance
            if scenario == 'hidden_standoff':
                view_open = True
            yaw = 2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
            if scenario == 'turn':
                yaw += 1.
            if scenario in ('rear_view', 'side_view'):
                yaw = math.atan2(-robot_y, 2.-robot_x) + (math.pi if scenario == 'rear_view' else math.pi/2)
            if scenario == 'occupied_arrival':
                local_blocked = True
            arrived_at = time.monotonic()
            publish()
            handle.succeed()
        return NavigateToPose.Result()

    def spin(handle):
        nonlocal yaw
        events.append('turn')
        if scenario == 'retry_budgets' and events.count('turn') <= 2:
            handle.abort()
            return Spin.Result()
        yaw += handle.request.target_yaw
        publish()
        handle.succeed()
        return Spin.Result()

    def locate(request, response):
        response.success = True
        response.centre = request.expected
        if scenario == 'refined':
            response.centre.y += 0.60
        return response

    node.create_service(LocateTree, '/husky1/locate_tree', locate)

    async def measure(request, response):
        nonlocal read_finished
        events.append('read')
        requests.append(request)
        scan_poses.append((robot_x, robot_y, yaw))
        if scenario == 'coasting':
            assert time.monotonic()-arrived_at >= 1.0
        if scenario in ('cancel_read', 'slow_read'):
            await read_wait
        response.success = True
        response.dbh_m = 0.0 if scenario == 'bad_reading' else 0.25 + 0.01 * request.tree_index
        if scenario == 'broad':
            target = request.tree.location
            bearings.append(math.atan2(robot_y - target.y, robot_x + 100. - target.x))
            response.dbh_m = .68 if len(requests) % 2 else .72
        if scenario in ('disagree', 'disagree_15_percent'):
            count = sum(r.tree_index == request.tree_index for r in requests)
            response.dbh_m = .72 if count == 1 else (.62 if scenario == 'disagree_15_percent' else .48)
        if scenario in ('retry_read', 'closer_view', 'current_pose_bad_view') and len(requests) == 1:
            response.success = False
            response.message = 'No ring near breast height'
        if scenario == 'retry_budgets' and len(requests) <= 2:
            response.success = False
        read_finished = True
        return response

    nav_server = ActionServer(node, NavigateToPose, '/husky1/navigate_to_pose', navigate,
                              cancel_callback=lambda _: CancelResponse.ACCEPT)
    spin_server = ActionServer(node, Spin, '/husky1/spin', spin)
    plan_server = ActionServer(node, ComputePathToPose, '/husky1/compute_path_to_pose', plan,
                               cancel_callback=lambda _: CancelResponse.ACCEPT)
    backup_server = ActionServer(node, BackUp, '/husky1/backup', backup,
                                 cancel_callback=lambda _: CancelResponse.ACCEPT)
    if scenario != 'no_service':
        node.create_service(MeasureDBH, '/husky1/measure_dbh', measure,
                            callback_group=ReentrantCallbackGroup())
    client = ActionClient(node, MeasureTrees, '/husky1/measure_trees')
    node.create_timer(0.025, publish)

    log_path = tmp_path / 'husky.log'
    with log_path.open('w') as log:
        # Keep the recovery scenarios' original geometry outside their configured range
        # The wide case exercises the new default without changing those failure triggers
        range_args = [] if scenario == 'wide_current_pose' else ['-p', 'tree_measurement_max_m:=2.5']
        process = subprocess.Popen([os.environ['HUSKY_EXECUTABLE'], '--ros-args',
                                    *range_args,
                                    '-p', 'waypoint_wait_timeout:=2.0',
                                    '-p', 'tree_progress_timeout_s:=0.5',
                                    '-p', 'dbh_timeout_s:=1.0'], stdout=log, stderr=log)

    def until(condition):
        deadline = time.monotonic() + 40
        while not condition():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic() < deadline, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=0.015)

    def result(future):
        until(future.done)
        return future.result()

    try:
        until(client.server_is_ready)
        goal = MeasureTrees.Goal()
        survey = goal.survey
        survey.tree_map.header.frame_id = 'world'
        survey.tree_map.header.stamp = node.get_clock().now().to_msg()
        single_tree = ('farther_pose', 'diverse_plans', 'shorter_path', 'goal_tolerance',
                       'rear_view', 'side_view', 'coasting', 'occupied_arrival', 'closer_view',
                       'new_retreat_position', 'same_retreat_position', 'stale_arrival', 'unobserved_arrival',
                       'current_pose', 'current_pose_bad_view', 'wide_current_pose')
        for x in ([] if scenario == 'empty' else [2.] if scenario in single_tree else [-3., 4., 2.]):
            tree = TreeLocation()
            tree.location.x = 100. + x
            survey.tree_map.trees.append(tree)
        if scenario == 'wide_current_pose':
            survey.tree_map.trees[0].location.x = 103.5
        if scenario in ('resume', 'invalid_resume'):
            index = 2 if scenario == 'resume' else 3
            survey.measurements = [TreeMeasurement(tree_index=index, dbh_m=0.50)]
        if scenario == 'current_pose_resume':
            survey.measurements = [TreeMeasurement(tree_index=i, dbh_m=.25) for i in (0, 1)]
        handle = result(client.send_goal_async(goal))
        if scenario == 'invalid_resume':
            assert not handle.accepted
            return
        assert handle.accepted
        outcome = handle.get_result_async()
        if scenario == 'manual_reposition':
            until(lambda: nav_goals)
            assert result(handle.cancel_goal_async()).goals_canceling
            until(lambda: nav_goals[0].is_cancel_requested)
            nav_wait.set_result(None)
            stopped = result(outcome)
            assert stopped.status == GoalStatus.STATUS_CANCELED
            robot_x = .5
            publish()
            goal.survey.measurements = stopped.result.measurements
            handle = result(client.send_goal_async(goal))
            assert handle.accepted
            outcome = handle.get_result_async()
            until(lambda: requests)
            assert requests[0].tree_index == 2
            assert len(nav_goals) == 1
            assert scan_poses[0][0] == pytest.approx(.5)
        if scenario in ('cancel_plan', 'cancel_backup'):
            goals = plan_goals if scenario == 'cancel_plan' else backup_goals
            until(lambda: goals)
            assert result(handle.cancel_goal_async()).goals_canceling
            until(lambda: goals[0].is_cancel_requested)
            assert not outcome.done()
            (plan_wait if scenario == 'cancel_plan' else backup_wait).set_result(None)
            assert result(outcome).status == GoalStatus.STATUS_CANCELED
            assert not requests
            return
        if scenario == 'stall_recover':
            until(lambda: nav_goals and nav_goals[0].is_cancel_requested)
            assert not backup_goals
            nav_wait.set_result(None)
        if scenario == 'cancel_nav':
            until(lambda: nav_goals)
            assert result(handle.cancel_goal_async()).goals_canceling
            until(lambda: nav_goals[0].is_cancel_requested)
            nav_wait.set_result(None)
            assert result(outcome).status == GoalStatus.STATUS_CANCELED
            assert events == ['drive']
            return
        if scenario == 'cancel_read':
            until(lambda: requests)
            assert result(handle.cancel_goal_async()).goals_canceling
            read_wait.set_result(True)
            finished = result(outcome)
            assert finished.status == GoalStatus.STATUS_CANCELED
            assert finished.result.measurements == [TreeMeasurement(tree_index=2, dbh_m=0.27)]
            assert events == ['drive', 'read']
            return
        finished = result(outcome)
        readings = finished.result.measurements
        assert 'turn' not in events
        if scenario in ('current_pose', 'current_pose_resume', 'current_pose_bad_view', 'wide_current_pose'):
            assert finished.status == GoalStatus.STATUS_SUCCEEDED, finished.result.message
            assert events[0] == 'read'
            assert scan_poses[0][:2] == (0., 0.)
            if scenario == 'wide_current_pose':
                assert requests[0].tree.location.x == 103.5
            if scenario == 'current_pose_bad_view':
                assert len(requests) == 2 and nav_goals
                assert math.dist(scan_poses[0][:2], scan_poses[1][:2]) >= .30
            else:
                assert not nav_goals and not plan_goals and not backup_goals
                assert len(requests) == 1
            assert len(readings) == (3 if scenario == 'current_pose_resume' else 1)
            if scenario == 'current_pose_resume':
                assert readings[:2] == survey.measurements
            return
        if scenario == 'manual_reposition':
            assert finished.status == GoalStatus.STATUS_SUCCEEDED and len(readings) == 3
            return
        if scenario in ('rear_view', 'side_view', 'coasting', 'closer_view', 'new_retreat_position'):
            assert finished.status == GoalStatus.STATUS_SUCCEEDED, finished.result.message
            assert len(readings) == 1
            if scenario == 'new_retreat_position':
                assert len(backup_goals) == 2
            if scenario == 'closer_view':
                assert len(requests) == 2
                assert math.hypot(robot_x-2., robot_y) == pytest.approx(1.35)
                assert math.dist(scan_poses[0][:2], scan_poses[1][:2]) >= 0.6
            return
        if scenario == 'stale_arrival':
            assert finished.status == GoalStatus.STATUS_ABORTED
            assert not backup_goals and not requests
            assert 'fresh costmaps' in finished.result.message
            return
        if scenario in ('occupied_arrival', 'same_retreat_position', 'unobserved_arrival'):
            assert finished.status == GoalStatus.STATUS_ABORTED
            assert len(backup_goals) == 1 and not requests
            assert 'reposition Husky' in finished.result.message
            return
        if scenario == 'wider_after_paths':
            assert finished.status == GoalStatus.STATUS_SUCCEEDED, finished.result.message
            assert len(readings) == 3 and not backup_goals
            assert len(plan_goals) > 24
            return
        if scenario in ('farther_pose', 'diverse_plans', 'shorter_path', 'goal_tolerance'):
            assert finished.status == GoalStatus.STATUS_SUCCEEDED, finished.result.message
            assert len(readings) == 1 and not backup_goals
            p = nav_goals[0].request.pose.pose.position
            if scenario == 'farther_pose':
                assert math.hypot(p.x-2., p.y) >= 2.0-1e-6
            if scenario == 'diverse_plans':
                assert math.hypot(p.x-2., p.y) == pytest.approx(2.15)
                for batch in (plan_goals[:8], plan_goals[8:16], plan_goals[16:24]):
                    sectors = {math.floor((math.atan2(g.request.goal.pose.position.y, g.request.goal.pose.position.x-2.)-math.pi)/(math.pi/4)+.5+1e-6)%8 for g in batch}
                    assert len(sectors) == 8
            if scenario == 'shorter_path':
                assert nav_goals[0].request.pose.pose.position != plan_goals[0].request.goal.pose.position
            if scenario == 'goal_tolerance':
                assert len(nav_goals) == 1
                assert math.hypot(robot_x-2., robot_y) == pytest.approx(2.45)
            return
        if scenario == 'slow_read':
            read_wait.set_result(True)
            until(lambda: read_finished)
            assert finished.status == GoalStatus.STATUS_ABORTED
            assert 'Timed out' in finished.result.message
            assert not readings
            return
        if scenario in ('stuck', 'backup_blocked'):
            assert finished.status == GoalStatus.STATUS_ABORTED
            assert 'reposition Husky' in finished.result.message
            assert not readings and len(nav_goals) < 8
            assert len(backup_goals) == 1
            return
        if scenario in ('disagree', 'disagree_15_percent'):
            assert finished.status == GoalStatus.STATUS_ABORTED
            assert not readings
            assert {r.tree_index for r in requests} == {0, 1, 2}
            return
        if scenario == 'blocked_tree':
            assert finished.status == GoalStatus.STATUS_ABORTED
            assert not readings and not events
            return
        if scenario in ('bad_reading', 'no_service'):
            assert finished.status == GoalStatus.STATUS_ABORTED
            assert not readings
            if scenario == 'bad_reading':
                assert set(r.tree_index for r in requests) == {0, 1, 2}
                assert len(requests) > 3
            else:
                assert events == ['drive']
            return
        assert finished.status == GoalStatus.STATUS_SUCCEEDED, finished.result.message
        if scenario == 'retry_budgets':
            assert [r.tree_index for r in requests] == [2, 2, 2, 1, 0]
            assert len(readings) == 3
            return
        if scenario in ('stall_recover', 'partial_backup', 'blocked_departure'):
            assert len(backup_goals) == 1
            assert len(readings) == 3
            if scenario == 'blocked_departure':
                assert len(plan_goals) >= 24
                return
        if scenario in ('plan_abort', 'path_collision', 'bad_path_end'):
            assert len(plan_goals) > len(nav_goals)
        if scenario == 'empty':
            assert not readings and not events
            return
        indices = [0, 1] if scenario == 'resume' else [2, 1, 0]
        if scenario == 'broad':
            assert [r.tree_index for r in requests] == [i for i in indices for _ in range(2)]
            for first, second in zip(bearings[::2], bearings[1::2]):
                assert abs(math.cos(second - first)) < 0.01
            assert all(r.dbh_m == pytest.approx(.70) for r in readings)
            return
        if scenario == 'retry_read':
            assert [r.tree_index for r in requests] == [2, *indices]
            assert [r.tree_index for r in readings] == indices
            assert math.dist(scan_poses[0][:2], scan_poses[1][:2]) >= 0.6
            return
        assert [r.tree_index for r in requests] == indices
        if scenario == 'refined':
            assert all(r.tree.location.y == pytest.approx(.60) for r in requests)
            assert [r.tree_index for r in readings] == indices
            return
        for request in requests:
            assert request.tree == survey.tree_map.trees[request.tree_index]
            assert request.map_header == survey.tree_map.header
        expected = list(survey.measurements) + [
            TreeMeasurement(tree_index=i, dbh_m=0.25 + 0.01 * i) for i in indices]
        assert readings == expected
        # Distant trees also get a coarse approach before choosing a measurement distance
        assert events.count('read') == len(indices)
        assert events.count('drive') >= len(indices)
        first = nav_goals[0].request.pose
        assert first.header.frame_id == 'husky1_map'
        trunk = survey.tree_map.trees[indices[0]].location.x - 100.
        distance = math.hypot(first.pose.position.x - trunk, first.pose.position.y)
        assert 1.5-1e-6 <= distance <= 2.5+1e-6
        if scenario in ('blocked_side', 'unknown_with_obstacle'):
            # Check clearance at the selected heading, Husky no longer has to face the trunk
            heading = 2*math.atan2(first.pose.orientation.z, first.pose.orientation.w)
            corners = [(x-first.pose.position.x, y-first.pose.position.y)
                       for x in (.75, 1.) for y in (.25, .5)]
            local_x = [math.cos(heading)*x+math.sin(heading)*y for x, y in corners]
            local_y = [-math.sin(heading)*x+math.cos(heading)*y for x, y in corners]
            assert (min(local_x) > .6 or max(local_x) < -.6 or
                    min(local_y) > .5 or max(local_y) < -.5)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        client.destroy()
        spin_server.destroy()
        plan_server.destroy()
        backup_server.destroy()
        nav_server.destroy()
        node.destroy_node()
        rclpy.shutdown()
