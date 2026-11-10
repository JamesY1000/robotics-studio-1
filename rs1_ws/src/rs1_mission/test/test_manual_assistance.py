"""Exercise the real coordinator with controlled executor results and live TF"""

import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rs1_interfaces.action import MeasureTrees, SampleROI, TraverseToROI, ReturnHome
from rs1_interfaces.msg import SafetyState, MissionStatus, TreeLocation, TreeMap, TreeMeasurement, TreeSurvey, PendingTreeMeasurement
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('failure', ['husky1', 'parrot1', 'reject'])
def test_assistance_retry_and_cancel(failure, tmp_path, monkeypatch):
    # A separate ROS domain keeps the absolute mission names away from real robots
    monkeypatch.setenv('ROS_DOMAIN_ID', '175')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('manual_assistance_test')
    safety = SafetyState(instance_id='test_safety')
    safety_pub = node.create_publisher(SafetyState, '/safety/state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(.05, lambda: safety_pub.publish(safety))
    broadcaster = TransformBroadcaster(node)
    goals = {'husky1': [], 'parrot1': []}
    reject_husky = failure == 'reject'
    statuses = []
    survey_updates = []
    node.create_subscription(TreeSurvey, '/mission/tree_survey', survey_updates.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(MissionStatus, '/mission/status', statuses.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    services = {name: node.create_client(Trigger, '/mission/' + name)
                for name in ('start', 'resume', 'cancel', 'manual_husky', 'manual_parrot',
                             'input_keyboard', 'input_controller')}
    servers = []
    async def return_home(handle):
        handle.succeed()
        return ReturnHome.Result(message='Reached home')

    for robot in goals:
        servers.append(ActionServer(node, ReturnHome, '/' + robot + '/return_home',
                                    execute_callback=return_home))
    for robot in goals:
        async def execute(handle, robot=robot):
            finished = Future()
            goals[robot].append((handle, finished))
            await finished
            return TraverseToROI.Result(message='test executor outcome')
        servers.append(ActionServer(
            node, TraverseToROI, '/' + robot + '/traverse_to_roi', execute_callback=execute,
            goal_callback=lambda _, robot=robot: (
                GoalResponse.REJECT if robot == 'husky1' and reject_husky
                else GoalResponse.ACCEPT),
            cancel_callback=lambda _: CancelResponse.ACCEPT))

    # The ground path handoff releases Parrot to start its sweep
    sweeps = []
    reject_sweep = False
    empty_sweep = False

    async def execute_sweep(handle):
        finished = Future()
        sweeps.append((handle, finished))
        await finished
        tree_map = TreeMap()
        tree_map.header.frame_id = 'world'
        for x in (() if empty_sweep else (-45., -39., -34.)):
            tree = TreeLocation()
            tree.location.x = x
            tree.location.y = 40.
            tree_map.trees.append(tree)
        return SampleROI.Result(tree_map=tree_map, tree_count=len(tree_map.trees))
    servers.append(ActionServer(
        node, SampleROI, '/parrot1/sample_roi', execute_callback=execute_sweep,
        goal_callback=lambda _: GoalResponse.REJECT if reject_sweep else GoalResponse.ACCEPT,
        cancel_callback=lambda _: CancelResponse.ACCEPT))

    surveys = []

    async def measure_trees(handle):
        finished = Future()
        surveys.append((handle, finished))
        readings = await finished
        if isinstance(readings, MeasureTrees.Result):
            return readings
        return MeasureTrees.Result(measurements=readings)

    servers.append(ActionServer(
        node, MeasureTrees, '/husky1/measure_trees', execute_callback=measure_trees,
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
        until(lambda: statuses and all(c.service_is_ready() for c in services.values()), 10)
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
            finish('parrot1', 'succeed')
            until(lambda: sweeps)
            assert not surveys
            if failure == 'husky1':
                # A finished map survives a pause while Husky is still following its ground path
                sweeps[-1][0].succeed()
                sweeps[-1][1].set_result(None)
                until(lambda: survey_updates and survey_updates[-1].tree_map.trees)
                assert not surveys
                assert call('manual_husky').success
                until(lambda: goals['husky1'][-1][0].is_cancel_requested)
                finish('husky1', 'canceled')
                until(lambda: state('manual_assistance', True))
                previous = len(goals['husky1'])
                assert call('resume').success
                until(lambda: len(goals['husky1']) > previous)
                assert goals['husky1'][-1][0].request.resume_route
                assert len(sweeps) == 1
            else:
                # Pause both actions, resume the route and restart the interrupted sweep
                assert call('manual_parrot').success
                until(lambda: goals['husky1'][-1][0].is_cancel_requested and
                      sweeps[-1][0].is_cancel_requested)
                finish('husky1', 'canceled')
                assert not call('resume').success
                sweeps[-1][0].canceled()
                sweeps[-1][1].set_result(None)
                until(lambda: state('manual_assistance', True))
                assert call('resume').success
                until(lambda: len(sweeps) == 2)
                assert goals['husky1'][-1][0].request.resume_route
            finish('husky1', 'succeed')
            until(lambda: state('sampling'))
            assert statuses[-1].control_mode == 'auto'
            traversal_counts = {r: len(v) for r, v in goals.items()}
            original_sweep = sweeps[-1][0].request
            for robot in (() if failure == 'husky1' else ('parrot', 'husky')):
                assert call('manual_' + robot).success
                until(lambda: state('manual_assistance', False))
                assert statuses[-1].control_mode == robot + '1'
                until(lambda: sweeps[-1][0].is_cancel_requested)
                sweep_count = len(sweeps)
                # Resume must not overwrite ownership of a still-cancelling goal.
                assert not call('resume').success
                assert len(sweeps) == sweep_count
                assert statuses[-1].control_mode == robot + '1'
                sweeps[-1][0].canceled()
                sweeps[-1][1].set_result(None)
                until(lambda: state('manual_assistance', True))

                # A rejected replacement must return to assistance and be retryable.
                reject_sweep = True
                revision = statuses[-1].input_revision
                assert call('resume').success
                until(lambda: state('manual_assistance', True) and
                      statuses[-1].input_revision > revision + 1)
                assert statuses[-1].control_mode == 'parrot1'
                assert len(sweeps) == sweep_count

                reject_sweep = False
                revision = statuses[-1].input_revision
                assert call('resume').success
                until(lambda: len(sweeps) == sweep_count + 1 and state('sampling'))
                assert statuses[-1].control_mode == 'auto'
                assert statuses[-1].input_revision > revision
                assert {r: len(v) for r, v in goals.items()} == traversal_counts
                resumed = sweeps[-1][0].request
                assert resumed.roi_min == original_sweep.roi_min
                assert resumed.roi_max == original_sweep.roi_max
                assert resumed.roi_origin.pose == original_sweep.roi_origin.pose
            if failure != 'husky1':
                # An empty sweep needs another look even when both robots have arrived
                empty_sweep = True
                sweep_count = len(sweeps)
                sweeps[-1][0].succeed()
                sweeps[-1][1].set_result(None)
                until(lambda: state('manual_assistance', True))
                assert not surveys
                empty_sweep = False
                assert call('resume').success
                until(lambda: len(sweeps) > sweep_count)
                assert {r: len(v) for r, v in goals.items()} == traversal_counts
                sweeps[-1][0].succeed()
                sweeps[-1][1].set_result(None)
            until(lambda: surveys and state('sampling', False))
            first, finished = surveys[0]
            assert [t.location.x for t in first.request.survey.tree_map.trees] == [-45., -39.]
            assert not first.request.survey.measurements
            assert not first.request.resume_from_current_pose
            first_reading = TreeMeasurement(tree_index=1, dbh_m=0.35)
            pending_view = PendingTreeMeasurement(tree_index=0, dbh_m=.68,
                trunk=Point(x=-45., y=40.), viewpoint=Point(x=-46.5, y=40.))
            assert call('manual_husky').success
            until(lambda: first.is_cancel_requested)
            first.canceled()
            finished.set_result(MeasureTrees.Result(measurements=[first_reading], pending_views=[pending_view]))
            until(lambda: state('manual_assistance', True))
            sweep_count = len(sweeps)
            assert call('resume').success
            until(lambda: len(surveys) == 2)
            resumed, finished = surveys[1]
            assert len(sweeps) == sweep_count
            assert resumed.request.survey.tree_map == first.request.survey.tree_map
            assert resumed.request.survey.measurements == [first_reading]
            assert resumed.request.resume_from_current_pose
            assert resumed.request.survey.pending_views == [pending_view]

            # An executor cannot silently drop an unfinished view or claim an incomplete survey is done
            resumed.succeed()
            finished.set_result([first_reading])
            until(lambda: state('manual_assistance', True))
            assert call('resume').success
            until(lambda: len(surveys) == 3)
            resumed, finished = surveys[-1]
            assert resumed.request.survey.measurements == [first_reading]
            assert resumed.request.survey.pending_views == [pending_view]

            # Another operator pause carries exactly the same progress through the coordinator
            assert call('manual_husky').success
            until(lambda: resumed.is_cancel_requested)
            resumed.canceled()
            finished.set_result(MeasureTrees.Result(measurements=[first_reading], pending_views=[pending_view]))
            until(lambda: state('manual_assistance', True))
            assert call('resume').success
            until(lambda: len(surveys) == 4)
            resumed, finished = surveys[-1]
            assert resumed.request.resume_from_current_pose
            assert resumed.request.survey.pending_views == [pending_view]
            resumed.succeed()
            finished.set_result([first_reading, TreeMeasurement(tree_index=0, dbh_m=0.70)])
            until(lambda: state('complete', True) and len(survey_updates[-1].measurements) == 2)
            assert survey_updates[-1].tree_map == first.request.survey.tree_map
            assert not survey_updates[-1].pending_views
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
