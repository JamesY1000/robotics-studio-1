"""Run the real executor with controlled TF and a stand-in Nav2 action server."""

import os
import signal
import subprocess
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rs1_interfaces.action import TraverseToROI
from rs1_interfaces.srv import SetGroundPath
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('scenario', [
    'fresh', 'recover_before_goal', 'recover_borderline', 'recover_after_abort',
    'recover_after_false_success',
    'timeout', 'cancel', 'mid_route_resume', 'real_failure', 'false_success',
])
def test_transform_readiness(scenario, tmp_path, monkeypatch):
    # Use a separate ROS domain so these fake goals and transforms cannot
    # interact with a user's running simulation.
    monkeypatch.setenv('ROS_DOMAIN_ID', '174')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('husky_transform_readiness_test')
    broadcaster = TransformBroadcaster(node)
    costmap_pub = node.create_publisher(
        OccupancyGrid, '/husky1/global_costmap/costmap',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    lag = 0.0 if scenario in (
        'fresh', 'recover_after_abort', 'recover_after_false_success',
        'mid_route_resume', 'real_failure', 'false_success') else 1.0
    if scenario == 'recover_borderline':
        # This passes DWB's 0.5s limit, but exceeds the local costmap's 0.3s
        # goal-transform limit. The mission must wait for this case too.
        lag = 0.35
    robot_x = 0.0
    goals = []
    feedback = []

    async def execute_navigation(goal_handle):
        # Let the test decide when Nav2 finishes, and whether it succeeded.
        finished = Future()
        goals.append((goal_handle, finished))
        await finished
        return NavigateThroughPoses.Result()

    cancel_calls = 0

    def cancel_navigation(_):
        nonlocal cancel_calls
        cancel_calls += 1
        return CancelResponse.REJECT if cancel_calls == 1 else CancelResponse.ACCEPT

    nav_server = ActionServer(
        node, NavigateThroughPoses, '/husky1/navigate_through_poses',
        execute_callback=execute_navigation, cancel_callback=cancel_navigation)
    mission = ActionClient(node, TraverseToROI, '/husky1/traverse_to_roi')
    handoff = node.create_client(SetGroundPath, '/husky1/set_ground_path')

    def publish():
        # A delayed map->odom transform models the observed handoff failure.
        # Odometry keeps updating, so only the map correction is old.
        now = node.get_clock().now()
        transforms = []
        for parent, child, stamp, x in (
            ('husky1_map', 'husky1_odom', now - Duration(seconds=lag), 0.0),
            ('husky1_odom', 'husky1_base_link', now, robot_x),
        ):
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.header.stamp = stamp.to_msg()
            transform.child_frame_id = child
            transform.transform.translation.x = x
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        broadcaster.sendTransform(transforms)

        grid = OccupancyGrid()
        grid.header.frame_id = 'husky1_map'
        grid.header.stamp = now.to_msg()
        grid.info.resolution = 0.5
        grid.info.width = 30
        grid.info.height = 10
        grid.info.origin.position.x = -2.0
        grid.info.origin.position.y = -2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        costmap_pub.publish(grid)

    node.create_timer(0.02, publish)
    log_path = tmp_path / 'husky.log'
    process = None

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=0.02)

    def until(condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, log_path.read_text()
            pump(0.02)

    def result(future):
        until(future.done)
        return future.result()

    def waiting():
        return any('Waiting for fresh localisation transform' in stage for stage in feedback)

    def finish_navigation(success):
        goal, finished = goals[-1]
        if success:
            goal.succeed()
        else:
            goal.abort()
        finished.set_result(None)

    try:
        # No retries are allowed: recovery can only pass if stale TF does not
        # consume a navigation retry. Shorten just the timeout test.
        timeout = 1.2 if scenario == 'timeout' else 10.0
        with log_path.open('w') as log:
            process = subprocess.Popen([
                os.environ['HUSKY_EXECUTABLE'], '--ros-args',
                '-p', 'use_sim_time:=false',
                '-p', 'waypoint_spacing:=' + ('2.0' if scenario == 'mid_route_resume' else '10.0'),
                '-p', 'max_chunk_retries:=0',
                '-p', 'waypoint_wait_timeout:=' + str(timeout),
            ], stdout=log, stderr=subprocess.STDOUT)
        until(lambda: mission.server_is_ready() and handoff.service_is_ready(), 10.0)
        pump(0.3)

        target = PoseStamped()
        target.header.frame_id = 'husky1_map'
        target.pose.position.x = 6.0
        target.pose.orientation.w = 1.0
        handle = result(mission.send_goal_async(
            TraverseToROI.Goal(target_roi=target, route_id='test-route'),
            feedback_callback=lambda msg: feedback.append(msg.feedback.stage)))
        assert handle.accepted
        outcome = handle.get_result_async()
        request = SetGroundPath.Request()
        request.path.header.frame_id = 'husky1_map'
        start = PoseStamped()
        start.header.frame_id = 'husky1_map'
        start.pose.orientation.w = 1.0
        request.path.poses = [start, target]
        started = time.monotonic()
        assert result(handoff.call_async(request)).accepted

        if scenario in ('recover_before_goal', 'recover_borderline', 'timeout', 'cancel'):
            until(waiting)
            pump(0.5)
            assert not goals, 'Stale TF must prevent sending any Nav2 goal'
            if scenario == 'timeout':
                response = result(outcome)
                assert response.status == GoalStatus.STATUS_ABORTED
                assert 'waypoint selection timed out' in response.result.message
                assert 1.0 <= time.monotonic() - started < 4.0
                return
            if scenario == 'cancel':
                assert result(handle.cancel_goal_async()).goals_canceling
                assert result(outcome).status == GoalStatus.STATUS_CANCELED
                stale = TraverseToROI.Goal(target_roi=target, route_id='other-mission', resume_route=True)
                assert not result(mission.send_goal_async(stale)).accepted
                retry = TraverseToROI.Goal(target_roi=target, route_id='test-route', resume_route=True)
                handle = result(mission.send_goal_async(retry))
                assert handle.accepted
                outcome = handle.get_result_async()
                # The remaining route runs without asking Parrot for another handoff
            lag = 0.0

        until(lambda: len(goals) == 1)
        if scenario == 'mid_route_resume':
            nav_goal, done = goals[0]
            assert [p.pose.position.x for p in nav_goal.request.poses] == [2., 4., 6.]
            robot_x = 2.
            nav_goal.publish_feedback(NavigateThroughPoses.Feedback(number_of_poses_remaining=2))
            pump(0.2)
            assert result(handle.cancel_goal_async()).goals_canceling
            until(lambda: nav_goal.is_cancel_requested)
            assert cancel_calls == 2
            nav_goal.canceled()
            done.set_result(None)
            assert result(outcome).status == GoalStatus.STATUS_CANCELED
            retry = TraverseToROI.Goal(target_roi=target, route_id='test-route', resume_route=True)
            handle = result(mission.send_goal_async(retry))
            assert handle.accepted
            outcome = handle.get_result_async()
            until(lambda: len(goals) == 2)
            assert [p.pose.position.x for p in goals[1][0].request.poses] == [4., 6.]
        if scenario in ('real_failure', 'false_success'):
            # A claimed success away from the ROI must still fail the
            # executor's independent arrival check, even with fresh TF.
            finish_navigation(scenario == 'false_success')
            response = result(outcome)
            assert response.status == GoalStatus.STATUS_ABORTED
            assert 'no further route progress after chunk retries' in response.result.message
            if scenario == 'false_success':
                assert 'outside the guidance pose arrival tolerance' in response.result.message
            return
        if scenario in ('recover_after_abort', 'recover_after_false_success'):
            # Let the previous fresh TF sample age out before Nav2 aborts.
            lag = 1.0
            pump(1.2)
            finish_navigation(scenario == 'recover_after_false_success')
            until(waiting)
            pump(0.5)
            assert len(goals) == 1
            assert not outcome.done(), 'A TF outage must not exhaust navigation retries'
            lag = 0.0
            until(lambda: len(goals) == 2)

        # Normal arrival still requires the robot's pose to reach the ROI.
        robot_x = 6.0
        pump(0.3)
        finish_navigation(True)
        response = result(outcome)
        assert response.status == GoalStatus.STATUS_SUCCEEDED
        assert response.result.message == 'Husky reached the ROI'
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        nav_server.destroy()
        mission.destroy()
        node.destroy_node()
        rclpy.shutdown()
