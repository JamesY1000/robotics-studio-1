"""Exercise command ownership and watchdogs through the real selector's ROS topics"""

import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from rs1_interfaces.msg import MissionStatus, SafetyState


@pytest.mark.parametrize('robot', ['husky1', 'parrot1'])
def test_command_ownership(robot, tmp_path):
    # Remap mission status too so this test cannot interact with a running mission
    namespace = '/selector_test_' + str(os.getpid())
    rclpy.init()
    node = rclpy.create_node('selector_test', namespace=namespace)
    safety = SafetyState(instance_id='test_safety')
    safety_pub = node.create_publisher(SafetyState, 'safety',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    send_safety = True
    node.create_timer(.04, lambda: safety_pub.publish(safety) if send_safety else None)
    status_pub = node.create_publisher(
        MissionStatus, 'status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    auto_pub = node.create_publisher(Twist, 'cmd_vel_combined' if robot == 'parrot1' else 'cmd_vel', 1)
    manual_pub = node.create_publisher(Twist, 'cmd_vel_manual', 1)
    received = []
    node.create_subscription(Twist, 'cmd_vel_controlled', received.append, 10)
    status = MissionStatus(instance_id='test', state='executing', control_mode='auto', manual_input='keyboard')
    automatic = Twist()
    automatic.linear.x = 0.2
    manual = Twist()
    manual.linear.x = 1.0
    manual.linear.y = 1.0
    manual.linear.z = 1.0
    manual.angular.z = 1.0
    send_status = True
    send_commands = True
    log_path = tmp_path / 'selector.log'
    process = None

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            if send_status:
                status_pub.publish(status)
            if send_commands:
                auto_pub.publish(automatic)
                manual_pub.publish(manual)
            rclpy.spin_once(node, timeout_sec=0.02)

    def expect(x, y=0.0, z=0.0, yaw=0.0):
        pump(0.5)
        received.clear()
        pump(0.15)
        assert len(received) >= 2
        for msg in received:
            assert (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z) == pytest.approx((x, y, z, yaw))

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                os.environ['VELOCITY_SELECTOR_EXECUTABLE'], '--ros-args',
                '-r', '__ns:=' + namespace, '-r', '/mission/status:=' + namespace + '/status',
                '-r', '/safety/state:=' + namespace + '/safety',
                '-p', 'robot:=' + robot, '-p', 'status_timeout:=0.4',
            ], stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 10
        while not received or not status_pub.get_subscription_count():
            assert time.monotonic() < deadline, log_path.read_text()
            pump(0.1)

        expect(0.2)  # Manual input cannot override auto
        safety.husky_stopped = safety.parrot_stopped = True
        safety.revision += 1
        expect(0.0)
        safety.husky_stopped = safety.parrot_stopped = False
        safety.revision += 1
        expect(0.2)
        send_safety = False
        pump(.6)
        expect(0.0)  # Lost safety authority blocks otherwise fresh Nav2 commands
        send_safety = True
        expect(0.2)
        status.state = 'manual_assistance'
        status.control_mode = robot
        expect(0.5, 0.5 if robot == 'parrot1' else 0.0,
               0.3 if robot == 'parrot1' else 0.0, 0.7)
        safety.husky_stopped = safety.parrot_stopped = True
        expect(0.0)  # All keyboard axes, including Parrot vertical motion, are stopped
        safety.husky_stopped = safety.parrot_stopped = False
        status.control_mode = 'parrot1' if robot == 'husky1' else 'husky1'
        expect(0.0)  # The unselected robot must hold
        status.control_mode = robot
        send_commands = False
        expect(0.0)  # Mode changes cannot replay old input
        send_commands = True
        send_status = False
        expect(0.0)  # Fresh input cannot bypass a missing coordinator heartbeat
        send_status = True
        manual.linear.x = float('nan')
        expect(0.0)
        manual.linear.x = 1.0
        status.state = 'executing'
        status.control_mode = 'auto'
        expect(0.2)
        send_commands = False
        expect(0.0)  # Input timeout also applies to autonomous commands
        status.state = 'complete'
        status.control_mode = 'hold'
        send_commands = True
        expect(0.0)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('state', ['executing', 'sampling', 'returning'])
def test_autonomous_states_forward_nav2(state, tmp_path):
    """Every autonomous mission phase must forward Nav2's command.

    Regression test: the ROI sweep publishes state "sampling", which used to
    fall through to "hold" and silently drop Nav2's command, stalling the
    parrot mid-sweep with "Failed to make progress".
    """
    namespace = '/selector_state_test_' + str(os.getpid()) + '_' + state
    rclpy.init()
    node = rclpy.create_node('selector_state_test', namespace=namespace)
    safety = SafetyState(instance_id='test_safety')
    safety_pub = node.create_publisher(SafetyState, 'safety',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    send_safety = True
    node.create_timer(.04, lambda: safety_pub.publish(safety) if send_safety else None)
    status_pub = node.create_publisher(
        MissionStatus, 'status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    auto_pub = node.create_publisher(Twist, 'cmd_vel_combined', 1)
    received = []
    node.create_subscription(Twist, 'cmd_vel_controlled', received.append, 10)
    status = MissionStatus(instance_id='test', state=state, control_mode='auto', manual_input='keyboard')
    automatic = Twist()
    automatic.linear.x = 0.2
    log_path = tmp_path / 'selector_state.log'
    process = None

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            status_pub.publish(status)
            auto_pub.publish(automatic)
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                os.environ['VELOCITY_SELECTOR_EXECUTABLE'], '--ros-args',
                '-r', '__ns:=' + namespace, '-r', '/mission/status:=' + namespace + '/status',
                '-r', '/safety/state:=' + namespace + '/safety',
                '-p', 'robot:=parrot1', '-p', 'status_timeout:=0.4',
            ], stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 10
        while not status_pub.get_subscription_count():
            assert time.monotonic() < deadline, log_path.read_text()
            pump(0.1)
        pump(0.4)
        received.clear()
        pump(0.3)
        assert len(received) >= 2
        for msg in received:
            assert msg.linear.x == pytest.approx(0.2)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
