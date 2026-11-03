"""Exercise the real stop authority, including reset races and restart behaviour"""

import os
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rs1_interfaces.msg import MissionStatus, SafetyState
from rs1_interfaces.srv import SafetyStop


def test_latched_stop_and_reset_handshake(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '183')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('safety_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    states = []
    node.create_subscription(SafetyState, '/safety/state', states.append, qos)
    publisher = node.create_publisher(MissionStatus, '/mission/status', qos)
    client = node.create_client(SafetyStop, '/safety/stop')
    mission = MissionStatus(instance_id='test_mission', control_mode='hold', cleanup_complete=True)
    send_mission = True
    node.create_timer(.03, lambda: publisher.publish(mission) if send_mission else None)
    log = (tmp_path/'safety.log').open('w')
    process = None

    def start():
        return subprocess.Popen([os.environ['SAFETY_EXECUTABLE']], stdout=log, stderr=log)

    def until(predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic()<deadline
            rclpy.spin_once(node, timeout_sec=.02)

    def pump(seconds=.1):
        end = time.monotonic()+seconds
        until(lambda: time.monotonic() >= end)

    def request(robot='all', stopped=True, observed=None):
        observed = observed or states[-1]
        future = client.call_async(SafetyStop.Request(robot=robot, stopped=stopped,
            expected_instance_id=observed.instance_id, expected_revision=observed.revision))
        until(future.done)
        pump()
        return future.result()

    def acknowledge():
        mission.safety_instance_id = states[-1].instance_id
        mission.safety_revision = states[-1].revision
        pump()

    def stop():
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    try:
        process = start()
        until(lambda: states and client.service_is_ready())
        assert states[-1].husky_stopped and states[-1].parrot_stopped
        assert not request(stopped=False).success  # No coordinator acknowledgement yet
        acknowledge()
        mission.cleanup_complete = False
        pump()
        assert not request(stopped=False).success
        mission.cleanup_complete = True
        mission.control_mode = 'auto'
        pump()
        assert not request(stopped=False).success
        mission.control_mode = 'hold'
        pump()
        assert request('husky1', False).success
        assert not states[-1].husky_stopped and states[-1].parrot_stopped
        acknowledge()
        assert request('parrot1', False).success
        assert not states[-1].husky_stopped and not states[-1].parrot_stopped
        assert request('husky1').success
        assert states[-1].husky_stopped and not states[-1].parrot_stopped
        old = states[-1]
        acknowledge()
        assert request('husky1').success  # A repeated stop supersedes an older reset
        acknowledge()
        assert not request('all', False, old).success
        assert not request('invalid').success
        send_mission = False
        pump(.9)
        assert not request(stopped=False).success
        assert request().success  # Stop works even when the coordinator is missing
        send_mission = True
        acknowledge()
        assert request(stopped=False).success
        old = states[-1]
        stop()
        process = start()
        until(lambda: states[-1].instance_id != old.instance_id)
        assert states[-1].husky_stopped and states[-1].parrot_stopped
        acknowledge()
        assert not request(stopped=False, observed=old).success
        before = len(states)
        pump(.3)
        assert len(states)-before >= 3  # Healthy state is a heartbeat, not just a retained message
    finally:
        if process and process.poll() is None:
            stop()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
