"""Real Husky and Parrot return actions with controlled Nav2 and changing map alignment"""

import copy
import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.task import Future
from rs1_interfaces.action import ReturnHome, TraverseToROI
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('robot', ['husky1', 'parrot1'])
@pytest.mark.parametrize('scenario', ['arrive', 'false_success', 'failure', 'reject', 'stall',
                                     'cancel', 'cancel_pending', 'already_home', 'stale'])
def test_home_executor(robot, scenario, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('home_executor_test')
    broadcaster = TransformBroadcaster(node)
    position = [12., 8.]
    map_yaw = .7
    lag = 0.
    goals = []
    accept = Future()

    async def request(_):
        if scenario == 'cancel_pending':
            await accept
        return GoalResponse.REJECT if scenario == 'reject' else GoalResponse.ACCEPT

    async def execute(handle):
        finished = Future()
        goals.append((handle, finished))
        await finished
        return NavigateToPose.Result()

    server = ActionServer(node, NavigateToPose, '/' + robot + '/navigate_to_pose',
                          execute_callback=execute, goal_callback=request,
                          callback_group=ReentrantCallbackGroup(),
                          cancel_callback=lambda _: CancelResponse.ACCEPT)
    client = ActionClient(node, ReturnHome, '/' + robot + '/return_home')
    outbound = ActionClient(node, TraverseToROI, '/' + robot + '/traverse_to_roi')

    def publish():
        transform = TransformStamped()
        transform.header.frame_id = 'world'
        transform.header.stamp = (node.get_clock().now()-Duration(seconds=lag)).to_msg()
        transform.child_frame_id = robot + '_map'
        transform.transform.translation.x = 5.
        transform.transform.rotation.z = math.sin(map_yaw/2)
        transform.transform.rotation.w = math.cos(map_yaw/2)
        body = TransformStamped()
        body.header.frame_id = robot + '_map'
        body.header.stamp = transform.header.stamp
        body.child_frame_id = robot + '_base_link'
        dx, dy = position[0]-5., position[1]
        body.transform.translation.x = math.cos(map_yaw)*dx + math.sin(map_yaw)*dy
        body.transform.translation.y = -math.sin(map_yaw)*dx + math.cos(map_yaw)*dy
        body.transform.rotation.w = 1.
        broadcaster.sendTransform([transform, body])

    node.create_timer(.03, publish)
    log_path = tmp_path/'executor.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([os.environ['HUSKY_EXECUTABLE' if robot == 'husky1' else 'PARROT_EXECUTABLE'],
                                    '--ros-args', '-p', 'home_progress_timeout_s:=0.9',
                                    '-p', 'home_timeout_s:=10.0'], stdout=log, stderr=log)

    def until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while not predicate():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic()<deadline, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.01)

    def result(future):
        until(future.done)
        return future.result()

    def pump(seconds):
        deadline = time.monotonic()+seconds
        until(lambda: time.monotonic() >= deadline, seconds+1)

    try:
        until(lambda: client.server_is_ready() and outbound.server_is_ready())
        pump(.3)
        goal = ReturnHome.Goal()
        goal.home.header.frame_id = 'world'
        goal.home.pose.position.x = 10.
        goal.home.pose.position.y = 8.
        goal.home.pose.orientation.w = 1.
        for invalid in ('frame', 'nan', 'quaternion'):
            bad = copy.deepcopy(goal)
            if invalid == 'frame':
                bad.home.header.frame_id = 'husky1_map'
            elif invalid == 'nan':
                bad.home.pose.position.x = math.nan
            else:
                bad.home.pose.orientation.w = 0.
            assert not result(client.send_goal_async(bad)).accepted
        if scenario == 'already_home':
            position[:] = [10., 8.]
            pump(.15)
        if scenario == 'stale':
            lag = 4.
            pump(2.2)
        handle = result(client.send_goal_async(goal))
        assert handle.accepted
        outcome = handle.get_result_async()
        if scenario in ('already_home', 'stale', 'reject'):
            response = result(outcome)
            expected = GoalStatus.STATUS_SUCCEEDED if scenario == 'already_home' else GoalStatus.STATUS_ABORTED
            assert response.status == expected
            assert not goals
            return

        if scenario == 'cancel_pending':
            pump(.3)
            assert result(handle.cancel_goal_async()).goals_canceling
            assert not outcome.done()
            accept.set_result(None)
        until(lambda: goals)
        nav, done = goals[-1]
        pose = nav.request.pose
        assert pose.header.frame_id == robot + '_map'
        assert pose.pose.position.x == pytest.approx(math.cos(map_yaw)*5+math.sin(map_yaw)*8)
        assert pose.pose.position.y == pytest.approx(-math.sin(map_yaw)*5+math.cos(map_yaw)*8)
        # Returning home reserves the executor, a phase 1 goal cannot preempt it
        target = copy.deepcopy(pose)
        target.header.frame_id = 'parrot1_map'
        assert not result(outbound.send_goal_async(TraverseToROI.Goal(target_roi=target))).accepted
        assert not result(client.send_goal_async(goal)).accepted

        if scenario in ('cancel', 'cancel_pending', 'stall'):
            if scenario == 'cancel':
                assert result(handle.cancel_goal_async()).goals_canceling
            until(lambda: nav.is_cancel_requested)
            # Keep the return action owned until Nav2 sends its terminal result
            pump(.2)
            assert not outcome.done()
            nav.canceled()
            done.set_result(None)
            response = result(outcome)
            assert response.status == (GoalStatus.STATUS_ABORTED if scenario == 'stall' else GoalStatus.STATUS_CANCELED)
            # A fresh attempt uses the same world home after a map-frame correction
            if scenario == 'cancel':
                map_yaw = -.4
                position[:] = [11., 8.]
                pump(.2)
                resumed = result(client.send_goal_async(goal))
                until(lambda: len(goals) == 2)
                nav, done = goals[-1]
                assert nav.request.pose.pose.position.x == pytest.approx(math.cos(map_yaw)*5+math.sin(map_yaw)*8)
                position[:] = [10., 8.]
                pump(.15)
                nav.succeed()
                done.set_result(None)
                assert result(resumed.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
            return
        if scenario == 'arrive':
            position[:] = [10., 8.]
            pump(.15)
        if scenario == 'failure':
            nav.abort()
        else:
            nav.succeed()
        done.set_result(None)
        response = result(outcome)
        assert response.status == (GoalStatus.STATUS_SUCCEEDED if scenario == 'arrive' else GoalStatus.STATUS_ABORTED)
    finally:
        if not accept.done():
            accept.set_result(None)
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()
