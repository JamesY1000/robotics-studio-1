"""Exercise the real coordinator with controlled executor results and live TF"""

import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rs1_interfaces.action import TraverseToROI
from rs1_interfaces.msg import MissionStatus
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('failure', ['husky1', 'parrot1', 'reject'])
def test_assistance_retry_and_cancel(failure, tmp_path, monkeypatch):
    # A separate ROS domain keeps the absolute mission names away from real robots
    monkeypatch.setenv('ROS_DOMAIN_ID', '175')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('manual_assistance_test')
    broadcaster = TransformBroadcaster(node)
    goals = {'husky1': [], 'parrot1': []}
    reject_husky = failure == 'reject'
    statuses = []
    node.create_subscription(MissionStatus, '/mission/status', statuses.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    services = {name: node.create_client(Trigger, '/mission/' + name)
                for name in ('start', 'resume', 'cancel', 'manual_husky', 'manual_parrot',
                             'input_keyboard', 'input_controller')}
    servers = []
    for robot in goals:
        async def execute(handle, robot=robot):
            finished = Future()
            goals[robot].append((handle, finished))
            await finished
            return TraverseToROI.Result(message='test executor outcome')
        servers.append(ActionServer(
            node, TraverseToROI, '/' + robot + '/traverse_to_roi', execute_callback=execute,
            goal_callback=lambda _, robot=robot: (
                GoalResponse.REJECT if robot == 'husky1' and reject_husky else GoalResponse.ACCEPT),
            cancel_callback=lambda _: CancelResponse.ACCEPT))

    tf_lag = 0.0

    def publish_tf():
        now = node.get_clock().now()
        from rclpy.duration import Duration
        transforms = []
        for child in ('husky1_base_link', 'parrot1_base_link', 'parrot1_map'):
            transform = TransformStamped()
            transform.header.frame_id = 'world'
            transform.header.stamp = (now - Duration(seconds=tf_lag)).to_msg()
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        broadcaster.sendTransform(transforms)

    node.create_timer(0.03, publish_tf)
    log_path = tmp_path / 'coordinator.log'
    process = None

    def until(condition, timeout=6.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic() < deadline, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=0.02)

    def call(name):
        future = services[name].call_async(Trigger.Request())
        until(future.done)
        return future.result()

    def state(name, clean=None):
        return bool(statuses and statuses[-1].state == name and
                    (clean is None or statuses[-1].cleanup_complete == clean))

    def finish(robot, outcome):
        handle, future = goals[robot][-1]
        getattr(handle, outcome)()
        future.set_result(None)

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                os.environ['COORDINATOR_EXECUTABLE'], '--ros-args',
                '-p', 'use_sim_time:=false', '-p', 'roi_x:=8.0', '-p', 'roi_y:=3.0',
            ], stdout=log, stderr=subprocess.STDOUT)
        until(lambda: all(client.service_is_ready() for client in services.values()) and statuses, 10)
        assert statuses[-1].manual_input == 'keyboard'
        assert not call('input_controller').success
        # Initial TF discovery can finish after service discovery
        deadline = time.monotonic() + 5
        while not call('start').success:
            assert time.monotonic() < deadline, log_path.read_text()
        if failure == 'reject':
            until(lambda: state('manual_assistance', True))
            assert statuses[-1].control_mode == 'husky1'
            assert not goals['parrot1']
            reject_husky = False
        else:
            until(lambda: all(goals.values()))
            finish(failure, 'abort')
            until(lambda: state('manual_assistance', False))
            assert statuses[-1].control_mode == failure
            other = 'husky1' if failure == 'parrot1' else 'parrot1'
            until(lambda: goals[other][-1][0].is_cancel_requested)
            assert not call('resume').success
            # A late success while stopping must not complete the mission
            finish(other, 'succeed')
            until(lambda: state('manual_assistance', True))

        assert call('input_controller').success
        until(lambda: statuses[-1].manual_input == 'controller')
        revision = statuses[-1].input_revision
        assert call('input_keyboard').success
        until(lambda: statuses[-1].manual_input == 'keyboard')
        assert statuses[-1].input_revision > revision

        # Stale localisation rejects Resume but keeps assistance and the saved ROI
        tf_lag = 4.0
        deadline = time.monotonic() + 2.3
        until(lambda: time.monotonic() >= deadline, 3)
        assert not call('resume').success
        assert state('manual_assistance', True)
        tf_lag = 0.0
        deadline = time.monotonic() + 0.3
        until(lambda: time.monotonic() >= deadline)
        counts = {robot: len(items) for robot, items in goals.items()}
        assert call('resume').success
        until(lambda: all(len(goals[r]) == counts[r] + 1 for r in goals))
        for items in goals.values():
            target = items[-1][0].request.target_roi
            assert target.pose.position.x == 8.0 and target.pose.position.y == 3.0
            assert target.header.frame_id == 'parrot1_map'

        assert call('manual_parrot').success
        until(lambda: state('manual_assistance', False))
        assert statuses[-1].control_mode == 'parrot1'
        assert call('manual_husky').success
        until(lambda: statuses[-1].control_mode == 'husky1')
        assert not call('resume').success
        for robot in goals:
            until(lambda robot=robot: goals[robot][-1][0].is_cancel_requested)
            finish(robot, 'canceled')
        until(lambda: state('manual_assistance', True))

        if failure == 'reject':
            assert call('cancel').success
            until(lambda: state('cancelled', True))
        else:
            counts = {r: len(v) for r, v in goals.items()}
            assert call('resume').success
            until(lambda: all(len(goals[r]) == counts[r] + 1 for r in goals))
            for robot in goals:
                finish(robot, 'succeed')
            until(lambda: state('complete', True))
        assert statuses[-1].control_mode == 'hold'
        assert not call('resume').success
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()
