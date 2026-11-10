"""Update Nikhil's phase 3 scenarios for confirmed DBHs and independent executor actions"""

import copy
import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rs1_interfaces.action import MeasureTrees, ReturnHome, SampleROI, TraverseToROI
from rs1_interfaces.msg import SafetyState, MissionStatus, TreeLocation, TreeMap, TreeMeasurement, TreeSurvey, PendingTreeMeasurement
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('scenario', ['complete', 'manual_twice', 'failure', 'reject', 'cancel',
                                     'late_success', 'moved_after_arrival', 'spawn_config', 'stale_resume', 'safety', 'safety_loss',
                                     'cancel_traversal', 'cancel_pending', 'cancel_sweep', 'cancel_map_ready',
                                     'cancel_measure', 'cancel_manual', 'cancel_safety', 'cancel_home_reject'])
def test_three_phases(scenario, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '180')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('return_home_coordinator_test')
    safety = SafetyState(instance_id='test_safety')
    safety_pub = node.create_publisher(SafetyState, '/safety/state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    send_safety = True
    node.create_timer(.05, lambda: safety_pub.publish(safety) if send_safety else None)
    broadcaster = TransformBroadcaster(node)
    positions = {'husky1': [20., -5.], 'parrot1': [22., -4.]}
    homes = copy.deepcopy(positions)
    if scenario == 'spawn_config':
        homes = {'husky1': [30., 4.], 'parrot1': [31., 6.]}
    lag = 0.
    statuses, surveys = [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MissionStatus, '/mission/status', statuses.append, qos)
    node.create_subscription(TreeSurvey, '/mission/tree_survey', surveys.append, qos)
    calls = {name: node.create_client(Trigger, '/mission/'+name)
             for name in ('start', 'resume', 'manual_husky', 'manual_parrot', 'cancel')}
    goals = {name: [] for name in ('husky1', 'parrot1', 'sweep', 'measure', 'home_husky1', 'home_parrot1')}
    reject_home = scenario in ('reject', 'cancel_home_reject')
    servers = []
    pending_acceptance = []

    def add(action, topic, key):
        async def accept(_):
            if scenario == 'cancel_pending' and key == 'husky1':
                released = Future()
                pending_acceptance.append(released)
                await released
            return GoalResponse.REJECT if key == 'home_husky1' and reject_home else GoalResponse.ACCEPT

        async def execute(handle):
            finished = Future()
            goals[key].append((handle, finished))
            return await finished
        servers.append(ActionServer(node, action, topic, execute_callback=execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            goal_callback=accept, callback_group=ReentrantCallbackGroup()))

    for robot in positions:
        add(TraverseToROI, '/'+robot+'/traverse_to_roi', robot)
        add(ReturnHome, '/'+robot+'/return_home', 'home_'+robot)
    add(SampleROI, '/parrot1/sample_roi', 'sweep')
    add(MeasureTrees, '/husky1/measure_trees', 'measure')

    def publish():
        transforms = []
        for child, xy in [('parrot1_map', [3., 2.])] + [(r+'_base_link', xy) for r, xy in positions.items()]:
            tf = TransformStamped()
            tf.header.frame_id = 'world'
            tf.header.stamp = (node.get_clock().now()-Duration(seconds=lag)).to_msg()
            tf.child_frame_id = child
            tf.transform.translation.x, tf.transform.translation.y = xy
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        broadcaster.sendTransform(transforms)

    node.create_timer(.03, publish)
    log_path = tmp_path/'coordinator.log'
    args = [os.environ['COORDINATOR_EXECUTABLE'], '--ros-args', '-p', 'roi_x:=8.0', '-p', 'roi_y:=3.0',
            '-p', 'roi_x_min:=6.0', '-p', 'roi_x_max:=10.0', '-p', 'roi_y_min:=1.0', '-p', 'roi_y_max:=5.0']
    if scenario == 'spawn_config':
        args += ['-p', 'home_from_start:=false', '-p', 'husky_home:=[30.0,4.0,0.0,0.3]',
                 '-p', 'parrot_home:=[31.0,6.0,4.0,0.8]']
    with log_path.open('w') as log:
        process = subprocess.Popen(args, stdout=log, stderr=log)

    def until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while not predicate():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic()<deadline, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.01)

    def pump(seconds):
        end = time.monotonic()+seconds
        until(lambda: time.monotonic()>=end, seconds+1)

    def call(name):
        future = calls[name].call_async(Trigger.Request())
        until(future.done)
        return future.result()

    def state(name, clean=None):
        return statuses and statuses[-1].state == name and (clean is None or statuses[-1].cleanup_complete == clean)

    def finish(key, action, outcome='succeed', **fields):
        handle, finished = goals[key][-1]
        getattr(handle, outcome)()
        finished.set_result(action.Result(**fields))

    def finish_home(robot, outcome='succeed'):
        if outcome == 'succeed':
            positions[robot] = list(homes[robot])
            pump(.1)
        finish('home_'+robot, ReturnHome, outcome, message='Home test result')

    def cancel_other(robot):
        key = 'home_'+robot
        until(lambda: goals[key] and goals[key][-1][0].is_cancel_requested)
        finish_home(robot, 'canceled')

    def cancel_and_return(active, readings=(), pending=()):
        nonlocal reject_home
        if scenario == 'cancel_manual':
            assert call('manual_husky').success
        assert call('cancel').success
        until(lambda: state('cancellation_requested') and statuses[-1].returning_home)
        assert statuses[-1].control_mode == 'hold'
        assert call('cancel').success
        assert not call('resume').success
        if scenario == 'cancel_pending':
            assert not goals['husky1'] and not goals['parrot1']
            pending_acceptance[0].set_result(None)
            until(lambda: goals['husky1'])
        if scenario == 'cancel_safety':
            safety.husky_stopped = True
            safety.revision += 1
            pump(.2)
        for key, action in active:
            until(lambda: goals[key][-1][0].is_cancel_requested)
            # No home action can overlap a previous executor, even when its result is delayed
            pump(.1)
            assert not goals['home_husky1'] and not goals['home_parrot1']
            fields = dict(measurements=list(readings), pending_views=list(pending)) if key == 'measure' else {}
            # Success can race a cancellation too, it must not restart the sweep
            outcome = 'succeed' if scenario == 'cancel_traversal' and key == 'parrot1' else 'canceled'
            finish(key, action, outcome, **fields)
        until(lambda: surveys[-1].measurements == list(readings) and surveys[-1].pending_views == list(pending))
        saved = copy.deepcopy(surveys[-1])
        work_counts = {key: len(goals[key]) for key in ('husky1', 'parrot1', 'sweep', 'measure')}
        if scenario == 'cancel_safety':
            until(lambda: state('manual_assistance', True))
            assert statuses[-1].control_mode == 'hold'
            assert not goals['home_husky1'] and not goals['home_parrot1']
            assert not call('resume').success
            safety.husky_stopped = False
            safety.revision += 1
            pump(.2)
            assert call('resume').success
        if scenario == 'cancel_home_reject':
            until(lambda: state('manual_assistance'))
            cancel_other('parrot1')
            until(lambda: state('manual_assistance', True))
            reject_home = False
            assert call('resume').success
        until(lambda: state('returning') and goals['home_husky1'] and goals['home_parrot1'])
        for robot in positions:
            target = goals['home_'+robot][-1][0].request.home
            assert [target.pose.position.x, target.pose.position.y] == homes[robot]
        assert call('cancel').success
        pump(.1)
        assert all(not goals['home_'+r][-1][0].is_cancel_requested for r in positions)

        # A navigation failure followed by another manual pause must preserve the cancelled survey
        for cycle in range(2):
            counts = {r: len(goals['home_'+r]) for r in positions}
            if cycle == 0:
                finish_home('husky1', 'abort')
            else:
                assert call('manual_husky').success
                cancel_other('husky1')
            until(lambda: state('manual_assistance'))
            assert not call('resume').success
            cancel_other('parrot1')
            until(lambda: state('manual_assistance', True))
            assert statuses[-1].returning_home
            assert call('cancel').success
            pump(.1)
            assert state('manual_assistance', True)
            positions['husky1'] = [12.+cycle, 6.]
            pump(.1)
            assert call('resume').success
            until(lambda: state('returning') and all(len(goals['home_'+r]) == counts[r]+1 for r in positions))
            assert surveys[-1] == saved
            assert {key: len(goals[key]) for key in work_counts} == work_counts
        finish_home('parrot1')
        until(lambda: statuses[-1].parrot_stage == 'Reached home')
        assert state('returning', False)
        finish_home('husky1')
        until(lambda: state('cancelled', True))
        assert 'both robots returned home' in statuses[-1].message
        assert statuses[-1].control_mode == 'hold'
        assert surveys[-1] == saved
        assert not call('resume').success
        assert not call('cancel').success

    try:
        until(lambda: statuses and all(client.service_is_ready() for client in calls.values()))
        deadline = time.monotonic()+5
        while not call('start').success:
            assert time.monotonic()<deadline, log_path.read_text()
        if scenario == 'cancel_pending':
            until(lambda: pending_acceptance)
            positions = {'husky1': [8., 3.], 'parrot1': [9., 4.]}
            cancel_and_return([('husky1', TraverseToROI)])
            assert not goals['parrot1'] and not goals['sweep']
            return
        until(lambda: goals['husky1'] and goals['parrot1'])
        if scenario == 'safety':
            safety.husky_stopped = True
            safety.revision += 1
            until(lambda: state('manual_assistance') and statuses[-1].control_mode == 'hold')
            assert not call('manual_husky').success and not call('resume').success
            for robot in positions:
                until(lambda: goals[robot][-1][0].is_cancel_requested)
                finish(robot, TraverseToROI, 'canceled')
            until(lambda: state('manual_assistance', True))
            safety.husky_stopped = False
            safety.revision += 1
            pump(.3)
            assert state('manual_assistance') and statuses[-1].control_mode == 'hold'
            assert len(goals['husky1']) == len(goals['parrot1']) == 1
            assert call('resume').success
            until(lambda: len(goals['husky1']) == len(goals['parrot1']) == 2)
        positions = {'husky1': [8., 3.], 'parrot1': [9., 4.]}
        if scenario == 'cancel_traversal':
            cancel_and_return([('husky1', TraverseToROI), ('parrot1', TraverseToROI)])
            assert not goals['sweep']
            return
        finish('parrot1', TraverseToROI)
        until(lambda: goals['sweep'])
        if scenario == 'cancel_sweep':
            cancel_and_return([('sweep', SampleROI), ('husky1', TraverseToROI)])
            assert not goals['measure']
            return
        tree_map = TreeMap(trees=[TreeLocation(location=Point(x=8., y=3.), tree_type='PINE_TREE')])
        if scenario.startswith('cancel_'):
            tree_map.trees.append(TreeLocation(location=Point(x=9., y=4.), tree_type='OAK_TREE'))
        tree_map.header.frame_id = 'world'
        finish('sweep', SampleROI, tree_map=tree_map, tree_count=1)
        pump(.2)
        assert not goals['measure'] and not goals['home_husky1']
        if scenario == 'cancel_map_ready':
            cancel_and_return([('husky1', TraverseToROI)])
            assert not goals['measure']
            return
        finish('husky1', TraverseToROI)
        until(lambda: goals['measure'])
        assert not goals['home_parrot1']
        if scenario in ('cancel_measure', 'cancel_manual', 'cancel_safety', 'cancel_home_reject'):
            reading = TreeMeasurement(tree_index=0, dbh_m=.25)
            view = PendingTreeMeasurement(tree_index=1, dbh_m=.7,
                trunk=Point(x=9., y=4.), viewpoint=Point(x=7., y=4.))
            if scenario == 'cancel_manual':
                # Preserve progress already held by the coordinator as well as results still in flight
                finish('measure', MeasureTrees, 'abort', measurements=[reading], pending_views=[view])
                until(lambda: state('manual_assistance', True))
                assert call('resume').success
                until(lambda: len(goals['measure']) == 2)
                assert goals['measure'][-1][0].request.survey.measurements == [reading]
                assert goals['measure'][-1][0].request.survey.pending_views == [view]
            cancel_and_return([('measure', MeasureTrees)], [reading], [view])
            return
        if scenario == 'safety':
            safety.parrot_stopped = True
            safety.revision += 1
            until(lambda: state('manual_assistance') and statuses[-1].control_mode == 'hold')
            until(lambda: goals['measure'][-1][0].is_cancel_requested)
            finish('measure', MeasureTrees, 'canceled', message='Interrupted for software stop')
            until(lambda: state('manual_assistance', True))
            assert not call('resume').success
            safety.parrot_stopped = False
            safety.revision += 1
            pump(.3)
            assert len(goals['measure']) == 1 and state('manual_assistance')
            assert call('resume').success
            until(lambda: len(goals['measure']) == 2)
        reading = TreeMeasurement(tree_index=0, dbh_m=.25)
        finish('measure', MeasureTrees, measurements=[reading])
        until(lambda: surveys[-1].measurements == [reading])
        saved_survey = copy.deepcopy(surveys[-1])

        if scenario == 'reject':
            until(lambda: state('manual_assistance'))
            cancel_other('parrot1')
            until(lambda: state('manual_assistance', True))
            reject_home = False
            assert call('resume').success
        until(lambda: goals['home_husky1'] and goals['home_parrot1'] and state('returning'))
        for robot in positions:
            home = goals['home_'+robot][-1][0].request.home
            assert home.header.frame_id == 'world'
            assert [home.pose.position.x, home.pose.position.y] == homes[robot]
        assert not state('complete')

        if scenario == 'cancel':
            assert call('cancel').success
            pump(.2)
            assert state('returning')
            assert all(not goals['home_'+r][-1][0].is_cancel_requested for r in positions)

        if scenario in ('manual_twice', 'failure', 'late_success', 'moved_after_arrival', 'stale_resume', 'safety', 'safety_loss'):
            if scenario == 'moved_after_arrival':
                finish_home('parrot1')
                until(lambda: statuses[-1].parrot_stage == 'Reached home')
            for cycle in range(2 if scenario in ('manual_twice', 'safety') else 1):
                counts = {r: len(goals['home_'+r]) for r in positions}
                if scenario == 'failure':
                    finish_home('husky1', 'abort')
                elif scenario == 'safety':
                    safety.husky_stopped = True
                    safety.revision += 1
                elif scenario == 'safety_loss':
                    send_safety = False
                else:
                    assert call('manual_husky').success
                until(lambda: state('manual_assistance'))
                assert not call('resume').success
                if scenario != 'failure':
                    cancel_other('husky1')
                if scenario == 'late_success':
                    until(lambda: goals['home_parrot1'][-1][0].is_cancel_requested)
                    finish_home('parrot1')
                elif scenario != 'moved_after_arrival':
                    cancel_other('parrot1')
                else:
                    # Selecting that robot and moving it means it needs another home goal
                    assert call('manual_parrot').success
                    positions['parrot1'] = [15., 7.]
                until(lambda: state('manual_assistance', True))
                if scenario == 'stale_resume':
                    lag = 4.
                    pump(2.2)
                    assert not call('resume').success
                    assert state('manual_assistance')
                    lag = 0.
                    pump(.2)
                if scenario in ('safety', 'safety_loss'):
                    assert statuses[-1].control_mode == 'hold'
                    assert not call('manual_parrot').success and not call('resume').success
                    safety.husky_stopped = False
                    safety.revision += 1
                    send_safety = True
                    pump(.3)
                    assert state('manual_assistance') and statuses[-1].control_mode == 'hold'
                    assert len(goals['home_husky1']) == counts['husky1']
                positions['husky1'] = [12.+cycle, 6.]
                pump(.1)
                assert call('resume').success
                until(lambda: len(goals['home_husky1']) == counts['husky1']+1 and state('returning'))
                if scenario == 'late_success':
                    assert len(goals['home_parrot1']) == counts['parrot1']
                else:
                    until(lambda: len(goals['home_parrot1']) == counts['parrot1']+1)
                assert surveys[-1] == saved_survey
                assert len(goals['sweep']) == 1
                assert len(goals['measure']) == (2 if scenario == 'safety' else 1)
                assert goals['home_husky1'][-1][0].request.home.pose.position.x == homes['husky1'][0]

        if scenario != 'late_success':
            finish_home('parrot1')
        until(lambda: statuses[-1].parrot_stage == 'Reached home')
        assert state('returning')
        finish_home('husky1')
        until(lambda: state('complete', True))
        assert statuses[-1].control_mode == 'hold'
        assert surveys[-1] == saved_survey
    finally:
        # A failed delayed-acceptance check must not leave the server waiting during teardown
        for waiting in pending_acceptance:
            if not waiting.done():
                waiting.set_result(None)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.01)
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
