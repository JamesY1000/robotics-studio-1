"""AGB uses actual confirmed survey readings, including after assistance and resume"""
import copy
import importlib.util
import math
import sys
import time
from pathlib import Path

import pytest

from rs1_interfaces.msg import PendingTreeMeasurement, TreeLocation, TreeMeasurement, TreeSurvey

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('carbon_calculator', Path(__file__).parents[1]/'scripts/carbon_calculator_node.py')
carbon = importlib.util.module_from_spec(spec)
spec.loader.exec_module(carbon)
MODELS = {'oak': (.71, 12.), 'pine': (.545, 10.)}


def survey():
    value = TreeSurvey(mission_id='mission-1')
    value.tree_map.header.frame_id = 'world'
    value.tree_map.header.stamp.sec = 10
    value.tree_map.trees = [TreeLocation(tree_type='OAK_TREE'), TreeLocation(tree_type='PINE_TREE')]
    return value


def estimate(value):
    return carbon.estimate_survey(value, MODELS, .0673, .976)


def test_only_confirmed_readings_count_and_resume_is_idempotent():
    value = survey()
    value.pending_views = [PendingTreeMeasurement(tree_index=0, dbh_m=.68)]
    empty, readings = estimate(value)
    assert empty.valid and empty.total_agb_kg == 0 and not readings
    assert empty.tree_count == 2 and empty.measured_count == 0
    value.measurements = [TreeMeasurement(tree_index=1, dbh_m=.13)]
    partial, readings = estimate(value)
    assert partial.measured_count == partial.estimated_count == 1
    assert readings[0].dbh_m == .13 and not readings[0].dbh_is_default
    expected_pine = .0673 * (.545 * 13**2 * 10)**.976
    assert partial.total_agb_kg == pytest.approx(expected_pine)
    assert estimate(copy.deepcopy(value))[0] == partial

    value.pending_views = []
    value.measurements.append(TreeMeasurement(tree_index=0, dbh_m=.70))
    complete, readings = estimate(value)
    assert complete.estimated_count == complete.tree_count == 2
    assert complete.total_agb_kg == pytest.approx(expected_pine + .0673 * (.71 * 70**2 * 12)**.976)
    assert complete.mission_id == value.mission_id and complete.header == value.tree_map.header
    assert estimate(copy.deepcopy(value))[0] == complete


@pytest.mark.parametrize('dbh', [0., -.1, math.nan, math.inf, 1e200])
def test_invalid_diameter_cannot_produce_a_total(dbh):
    value = survey()
    value.measurements = [TreeMeasurement(tree_index=0, dbh_m=dbh)]
    summary, readings = estimate(value)
    assert not summary.valid and not readings and summary.total_agb_kg == 0


@pytest.mark.parametrize('indices', [[0, 0], [0, 2]])
def test_duplicate_or_wrong_tree_index_rejects_the_snapshot(indices):
    value = survey()
    value.measurements = [TreeMeasurement(tree_index=i, dbh_m=.3) for i in indices]
    summary, readings = estimate(value)
    assert not summary.valid and not readings


def test_unknown_species_has_no_invented_model_or_default_dbh():
    value = survey()
    value.tree_map.trees[0].tree_type = 'UNKNOWN'
    value.measurements = [TreeMeasurement(tree_index=0, dbh_m=.3)]
    summary, readings = estimate(value)
    assert summary.valid and summary.measured_count == 1 and summary.estimated_count == 0
    assert summary.total_agb_kg == 0 and not readings
    assert 'no species model' in summary.message


def test_live_calculator_to_dashboard_and_new_mission_reset(monkeypatch):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rs1_interfaces.msg import MissionStatus, TreeMap
    from std_srvs.srv import Trigger
    sys.path.insert(0, str(ROOT/'mission_control/backend'))
    from mission_control.ros_adapter import RosAdapter

    monkeypatch.setenv('ROS_DOMAIN_ID', '178')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('agb_integration_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    survey_pub = node.create_publisher(TreeSurvey, '/mission/tree_survey', qos)
    map_pub = node.create_publisher(TreeMap, '/mission/tree_map', qos)
    status_pub = node.create_publisher(MissionStatus, '/mission/status', qos)
    value = survey()
    status = MissionStatus(instance_id=value.mission_id, state='sampling')
    status.roi.header.frame_id = 'world'
    status_pub.publish(status)
    map_pub.publish(value.tree_map)
    survey_pub.publish(value)
    calculator = carbon.CarbonCalculator()
    adapter = RosAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(calculator)

    def until(predicate):
        deadline = time.monotonic()+8
        while not predicate():
            assert time.monotonic()<deadline, adapter.snapshot()
            executor.spin_once(timeout_sec=.02)

    try:
        until(lambda: adapter.snapshot().get('biomass') is not None)
        assert adapter.snapshot()['total_agb_kg'] is None
        value.measurements = [TreeMeasurement(tree_index=1, dbh_m=.13)]
        value.pending_views = [PendingTreeMeasurement(tree_index=0, dbh_m=.68)]
        survey_pub.publish(value)
        until(lambda: adapter.snapshot()['biomass']['estimated_count'] == 1)
        first = adapter.snapshot()['total_agb_kg']
        assert first == pytest.approx(estimate(value)[0].total_agb_kg)
        client = node.create_client(Trigger, '/calculate_carbon')
        until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success
        assert adapter.snapshot()['total_agb_kg'] == first
        value.pending_views = []
        value.measurements.append(TreeMeasurement(tree_index=0, dbh_m=.70))
        survey_pub.publish(value)
        until(lambda: adapter.snapshot()['biomass']['estimated_count'] == 2)
        assert adapter.snapshot()['total_agb_kg'] == pytest.approx(estimate(value)[0].total_agb_kg)

        # The calculator may still hold an old latched result while the next coordinator starts
        status.instance_id = 'mission-2'
        status.state = 'ready'
        status_pub.publish(status)
        until(lambda: adapter.snapshot()['mission']['instance_id'] == 'mission-2')
        assert adapter.snapshot()['total_agb_kg'] is None
        value = survey()
        value.mission_id = 'mission-2'
        value.tree_map.header.stamp.sec = 20
        map_pub.publish(value.tree_map)
        survey_pub.publish(value)
        until(lambda: adapter.snapshot()['biomass'] is not None)
        assert adapter.snapshot()['biomass']['measured_count'] == 0
        assert adapter.snapshot()['total_agb_kg'] is None
    finally:
        adapter.close()
        executor.shutdown()
        calculator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
