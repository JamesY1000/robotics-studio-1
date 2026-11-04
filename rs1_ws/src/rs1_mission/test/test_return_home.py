"""Exercise the real coordinator through ROI, sampling and the home leg"""

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
from rs1_interfaces.action import SampleROI, TraverseToROI
from rs1_interfaces.msg import MissionStatus
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

ROI = (8.0, 3.0)
HOME = (20.0, -5.0)


@pytest.mark.parametrize(
    'scenario', ['complete', 'resume', 'manual', 'cancel', 'reject', 'sample_manual', 'sample_cancel',
                 'roi_cancel', 'cancel_twice', 'cancel_resume', 'cancel_retry', 'home_manual_cancel'])
def test_return_home(scenario, tmp_path, monkeypatch):
    # A separate ROS domain keeps the absolute mission names away from real robots
    monkeypatch.setenv('ROS_DOMAIN_ID', '176')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('return_home_test')
    broadcaster = TransformBroadcaster(node)
    goals = {'husky1': [], 'parrot1': []}
    sample_goals = []
    reject_husky = False
    statuses = []
    node.create_subscription(MissionStatus, '/mission/status', statuses.append,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    services = {name: node.create_client(Trigger, '/mission/' + name)
                for name in ('start', 'resume', 'cancel', 'manual_husky', 'manual_parrot')}
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

    async def execute_sample(handle):
        finished = Future()
        sample_goals.append((handle, finished))
        await finished
        return SampleROI.Result(message='test sampling outcome')
    servers.append(ActionServer(node, SampleROI, '/parrot1/sample_roi', execute_callback=execute_sample,
                                cancel_callback=lambda _: CancelResponse.ACCEPT))

    def publish_tf():
        transforms = []
        for child in ('husky1_base_link', 'parrot1_base_link', 'parrot1_map'):
            transform = TransformStamped()
            transform.header.frame_id = 'world'
            transform.header.stamp = node.get_clock().now().to_msg()
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

    def finish(items, outcome):
        handle, future = items[-1]
        getattr(handle, outcome)()
        future.set_result(None)

    def assert_targets(target):
        for robot in goals:
            pose = goals[robot][-1][0].request.target_roi
            assert (pose.pose.position.x, pose.pose.position.y) == target
            assert pose.header.frame_id == 'parrot1_map'

    def wait_for_new_goals(counts):
        until(lambda: all(len(goals[r]) == counts[r] + 1 for r in goals))

    def cancel_active(items):
        until(lambda: items[-1][0].is_cancel_requested)
        finish(items, 'canceled')

    def assert_cancel_return():
        assert_targets(HOME)
        until(lambda: state('executing') and statuses[-1].cancel_return)
        assert statuses[-1].message == 'Mission cancelled; returning home'
        assert statuses[-1].control_mode == 'auto'

    def finish_cancel_return():
        finish(goals['parrot1'], 'succeed')
        finish(goals['husky1'], 'succeed')
        until(lambda: state('cancelled', True))
        assert statuses[-1].message == 'Mission cancelled: both robots returned home'
        assert statuses[-1].husky_stage == 'Reached home'
        assert statuses[-1].control_mode == 'hold'
        assert not any(status.state == 'complete' for status in statuses)

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                os.environ['COORDINATOR_EXECUTABLE'], '--ros-args', '-p', 'use_sim_time:=false',
                '-p', f'roi_x:={ROI[0]}', '-p', f'roi_y:={ROI[1]}',
                '-p', 'roi_x_min:=6.0', '-p', 'roi_y_min:=1.0',
                '-p', 'roi_x_max:=10.0', '-p', 'roi_y_max:=5.0',
                '-p', f'home_x:={HOME[0]}', '-p', f'home_y:={HOME[1]}',
            ], stdout=log, stderr=subprocess.STDOUT)
        until(lambda: all(client.service_is_ready() for client in services.values()) and statuses, 10)
        # Initial TF and action discovery can finish after service discovery
        deadline = time.monotonic() + 5
        while not call('start').success:
            assert time.monotonic() < deadline, log_path.read_text()

        # Phase 1: same order as always, and it hands over to sampling
        until(lambda: all(goals.values()))
        assert_targets(ROI)

        if scenario in ('roi_cancel', 'cancel_twice', 'cancel_resume', 'cancel_retry'):
            # Cancel stops the outbound goals, holds the robots, then sends them home
            counts = {r: len(v) for r, v in goals.items()}
            assert call('cancel').success
            until(lambda: state('cancellation_requested'))
            assert statuses[-1].control_mode == 'hold'
            for robot in goals:
                cancel_active(goals[robot])
            wait_for_new_goals(counts)
            assert_cancel_return()
            assert not sample_goals
            if scenario == 'cancel_twice':
                # A second Cancel must not interrupt the robots on their way home
                counts = {r: len(v) for r, v in goals.items()}
                assert call('cancel').success
                assert_cancel_return()
                assert not any(goals[r][-1][0].is_cancel_requested for r in goals)
                assert {r: len(v) for r, v in goals.items()} == counts
            if scenario == 'cancel_retry':
                # Cancel never strands the robots: after a failure it tries home again
                finish(goals['parrot1'], 'abort')
                until(lambda: state('manual_assistance', False))
                cancel_active(goals['husky1'])
                until(lambda: state('manual_assistance', True))
                counts = {r: len(v) for r, v in goals.items()}
                assert call('cancel').success
                wait_for_new_goals(counts)
                assert_cancel_return()
            if scenario == 'cancel_resume':
                # A failure on that leg still resumes home and still ends as cancelled
                finish(goals['husky1'], 'abort')
                until(lambda: state('manual_assistance', False))
                assert statuses[-1].cancel_return
                cancel_active(goals['parrot1'])
                until(lambda: state('manual_assistance', True))
                counts = {r: len(v) for r, v in goals.items()}
                assert call('resume').success
                wait_for_new_goals(counts)
                assert_cancel_return()
            finish_cancel_return()
            return

        finish(goals['parrot1'], 'succeed')
        finish(goals['husky1'], 'succeed')
        until(lambda: sample_goals and statuses[-1].message.startswith('Sampling'))
        # The velocity selector only passes Parrot's sweep commands in this exact state
        assert state('executing')
        assert statuses[-1].control_mode == 'auto'

        if scenario == 'sample_manual':
            # The dashboard offers assistance while executing, so sampling must honour it
            assert call('manual_parrot').success
            until(lambda: state('manual_assistance', False))
            assert statuses[-1].control_mode == 'parrot1'
            until(lambda: sample_goals[-1][0].is_cancel_requested)
            finish(sample_goals, 'canceled')
            until(lambda: state('manual_assistance', True))
            return
        if scenario == 'sample_cancel':
            counts = {r: len(v) for r, v in goals.items()}
            assert call('cancel').success
            cancel_active(sample_goals)
            wait_for_new_goals(counts)
            assert_cancel_return()
            finish_cancel_return()
            return

        # Phase 3: reaches the home pose without finishing the mission early
        counts = {r: len(v) for r, v in goals.items()}
        if scenario == 'reject':
            reject_husky = True
        finish(sample_goals, 'succeed')
        if scenario == 'reject':
            until(lambda: state('manual_assistance', True))
            assert 'home' in statuses[-1].message
            assert statuses[-1].control_mode == 'husky1'
            assert len(goals['parrot1']) == counts['parrot1']
            reject_husky = False
            assert call('resume').success
        wait_for_new_goals(counts)
        assert_targets(HOME)
        until(lambda: state('executing'))
        assert statuses[-1].message == 'Returning home'
        assert statuses[-1].control_mode == 'auto'
        assert not statuses[-1].cancel_return

        if scenario == 'resume':
            # A failure on the way home must not send the robots back to the ROI
            finish(goals['husky1'], 'abort')
            until(lambda: state('manual_assistance', False))
            assert 'home' in statuses[-1].message
            until(lambda: goals['parrot1'][-1][0].is_cancel_requested)
            finish(goals['parrot1'], 'canceled')
            until(lambda: state('manual_assistance', True))
            counts = {r: len(v) for r, v in goals.items()}
            assert call('resume').success
            wait_for_new_goals(counts)
            assert_targets(HOME)
        elif scenario == 'manual':
            # The operator can take over on the home leg and continue it
            assert call('manual_parrot').success
            until(lambda: state('manual_assistance', False))
            assert statuses[-1].control_mode == 'parrot1'
            for robot in goals:
                until(lambda robot=robot: goals[robot][-1][0].is_cancel_requested)
                finish(goals[robot], 'canceled')
            until(lambda: state('manual_assistance', True))
            counts = {r: len(v) for r, v in goals.items()}
            assert call('resume').success
            wait_for_new_goals(counts)
            assert_targets(HOME)
        elif scenario == 'cancel':
            # Already going home: the robots carry on, and the mission ends as cancelled
            counts = {r: len(v) for r, v in goals.items()}
            assert call('cancel').success
            assert_cancel_return()
            assert not any(goals[r][-1][0].is_cancel_requested for r in goals)
            assert {r: len(v) for r, v in goals.items()} == counts
            finish_cancel_return()
            return
        elif scenario == 'home_manual_cancel':
            # A failed home leg sits in assistance; Cancel must not end it there
            finish(goals['husky1'], 'abort')
            until(lambda: state('manual_assistance', False))
            cancel_active(goals['parrot1'])
            until(lambda: state('manual_assistance', True))
            counts = {r: len(v) for r, v in goals.items()}
            assert call('cancel').success
            wait_for_new_goals(counts)
            assert_cancel_return()
            finish_cancel_return()
            return

        # Parrot finishes after its handoff; only Husky's arrival completes the mission
        finish(goals['parrot1'], 'succeed')
        until(lambda: statuses[-1].parrot_stage == 'Reached home')
        assert state('executing')
        finish(goals['husky1'], 'succeed')
        until(lambda: state('complete', True))
        assert statuses[-1].husky_stage == 'Reached home'
        assert statuses[-1].control_mode == 'hold'
        assert len(sample_goals) == 1
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
