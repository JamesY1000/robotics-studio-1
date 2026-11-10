"""Run the real Twist converter and selector with controlled PS4 input"""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from rs1_interfaces.msg import MissionStatus, SafetyState
from sensor_msgs.msg import Joy


@pytest.mark.parametrize('robot', ['husky1', 'parrot1'])
def test_exclusive_input_and_rearming(robot, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '177')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('controller_input_test')
    safety = SafetyState(instance_id='test_safety')
    safety_pub = node.create_publisher(SafetyState, '/safety/state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    send_safety = True
    node.create_timer(.04, lambda: safety_pub.publish(safety) if send_safety else None)
    status_pub = node.create_publisher(MissionStatus, '/mission/status',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    keyboard_pub = node.create_publisher(Twist, '/' + robot + '/cmd_vel_manual', 1)
    auto_pub = node.create_publisher(Twist, '/' + robot + (
        '/cmd_vel_combined' if robot == 'parrot1' else '/cmd_vel'), 1)
    joy_pub = node.create_publisher(Joy, '/teleop/joy', 1)
    controller_pub = node.create_publisher(Twist, '/teleop/cmd_vel_controller', 1)
    received = []
    node.create_subscription(Twist, '/' + robot + '/cmd_vel_controlled', received.append, 10)
    status = MissionStatus(instance_id='controller_test', state='manual_assistance',
                           control_mode=robot, manual_input='keyboard')
    keyboard = Twist()
    keyboard.linear.x = 0.1
    automatic = Twist()
    automatic.linear.x = 0.2
    joy = Joy(axes=[0.6, 0.8, 0.4, 0.2, 0.0, 0.0], buttons=[0] * 15)
    joy.buttons[9] = 1
    send_joy = True
    rogue_controller = False
    selector = None
    converter = None
    log_path = tmp_path / 'controller.log'
    config = Path(__file__).resolve().parents[1] / 'config/ps4_teleop.yaml'

    def start_converter(log):
        executable = get_package_prefix('teleop_twist_joy') + '/lib/teleop_twist_joy/teleop_node'
        return subprocess.Popen([
            executable, '--ros-args', '-r', '__ns:=/teleop',
            '-r', 'cmd_vel:=cmd_vel_controller', '--params-file', str(config),
        ], stdout=log, stderr=subprocess.STDOUT)

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert selector.poll() is None, log_path.read_text()
            status_pub.publish(status)
            keyboard_pub.publish(keyboard)
            auto_pub.publish(automatic)
            if send_joy:
                joy.header.stamp = node.get_clock().now().to_msg()
                joy_pub.publish(joy)
            if rogue_controller:
                controller_pub.publish(keyboard)
            rclpy.spin_once(node, timeout_sec=0.02)

    def expect(x=0.0, y=0.0, z=0.0, yaw=0.0):
        pump(0.45)
        received.clear()
        pump(0.15)
        assert len(received) >= 2, log_path.read_text()
        for msg in received:
            assert (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z) == pytest.approx((x, y, z, yaw))

    def driving():
        expect(0.4, 0.3 if robot == 'parrot1' else 0.0,
               0.06 if robot == 'parrot1' else 0.0, 0.28)

    def release_and_press():
        joy.buttons[9] = 0
        expect()
        joy.buttons[9] = 1
        driving()

    def stop(process):
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    try:
        with log_path.open('w') as log:
            selector = subprocess.Popen([
                os.environ['VELOCITY_SELECTOR_EXECUTABLE'], '--ros-args',
                '-r', '__ns:=/' + robot, '-p', 'robot:=' + robot,
            ], stdout=log, stderr=subprocess.STDOUT)
            converter = start_converter(log)
            deadline = time.monotonic() + 10
            while not received or joy_pub.get_subscription_count() < 2:
                assert converter.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                pump(0.1)

            expect(0.1)  # The controller cannot interrupt keyboard input
            status.manual_input = 'controller'
            status.input_revision += 1
            expect()  # Selecting a controller with L1 already held must not move
            release_and_press()

            status.manual_input = 'keyboard'
            status.input_revision += 1
            expect(0.1)
            status.manual_input = 'controller'
            status.input_revision += 1
            expect()
            release_and_press()

            safety.husky_stopped = safety.parrot_stopped = True
            safety.revision += 1
            expect()
            safety.husky_stopped = safety.parrot_stopped = False
            safety.revision += 1
            expect()  # Reset cannot rearm an already held L1
            release_and_press()
            send_safety = False
            pump(.6)
            expect()
            send_safety = True
            expect()
            release_and_press()

            # Even if an intermediate source selection was missed, its revision clears old input
            status.input_revision += 2
            expect()
            release_and_press()
            status.control_mode = 'parrot1' if robot == 'husky1' else 'husky1'
            expect()
            status.control_mode = robot
            expect()
            release_and_press()

            # A fresh Twist cannot replace a missing controller heartbeat
            send_joy = False
            rogue_controller = True
            expect()
            rogue_controller = False
            send_joy = True
            expect()
            release_and_press()

            # Losing the converter must also stop motion even while Joy keeps arriving
            stop(converter)
            expect()
            converter = start_converter(log)
            pump(0.5)
            expect()
            release_and_press()

            status.manual_input = 'auto'  # Invalid manual source must not admit autonomous input
            expect()
            status.manual_input = 'controller'
            status.state = 'executing'
            status.control_mode = 'auto'
            expect(0.2)
            status.state = 'complete'
            status.control_mode = 'hold'
            expect()
    finally:
        stop(converter)
        stop(selector)
        node.destroy_node()
        rclpy.shutdown()
