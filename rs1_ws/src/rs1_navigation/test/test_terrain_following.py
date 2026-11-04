"""Exercise the real terrain follower through its ROS topics, without Gazebo."""

import copy
import os
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def test_navigation_hover_transitions(tmp_path):
    # Give this test its own namespace so it cannot command a running robot.
    namespace = '/terrain_following_test_' + str(os.getpid())
    rclpy.init()
    node = rclpy.create_node('terrain_following_test', namespace=namespace)
    status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_pub = node.create_publisher(
        GoalStatusArray, 'navigate_to_pose/_action/status', status_qos)
    backup_pub = node.create_publisher(GoalStatusArray, 'backup/_action/status', status_qos)
    scan_pub = node.create_publisher(LaserScan, 'downwards_scan', qos_profile_sensor_data)
    cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    received = []
    node.create_subscription(Twist, 'cmd_vel_combined', received.append, 10)

    # A 3m reading asks the 4m clearance controller to climb. This makes an
    # unwanted height correction observable even when horizontal commands stop.
    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 30.0
    scan.ranges = [3.0, 3.0, 3.0]
    command = Twist()
    command.linear.x = 0.3
    command.linear.y = 0.1
    command.angular.z = 0.2
    publish_command = True
    goal_number = 0

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            scan.header.stamp = node.get_clock().now().to_msg()
            scan_pub.publish(scan)
            if publish_command:
                cmd_pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)

    def goal(status):
        nonlocal goal_number
        goal_number += 1
        msg = GoalStatus()
        msg.goal_info.goal_id.uuid = [goal_number] + [0] * 15
        msg.goal_info.stamp = node.get_clock().now().to_msg()
        msg.status = status
        return msg

    def send_status(*goals):
        status_pub.publish(GoalStatusArray(status_list=list(goals)))
        pump(0.15)

    def expect_motion(x, y, z, yaw):
        # Allow callbacks to settle, then check several outputs rather than
        # accepting one old message left in a subscriber queue.
        pump(0.3)
        received.clear()
        pump(0.2)
        assert len(received) >= 2, 'The node must keep publishing in hover too'
        for msg in received:
            actual = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
            assert all(abs(a - b) < 1e-6 for a, b in zip(actual, (x, y, z, yaw))), actual

    log_path = tmp_path / 'terrain_following.log'
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                os.environ['TERRAIN_FOLLOWING_EXECUTABLE'], '--ros-args',
                '-r', '__ns:=' + namespace,
                '-p', 'use_sim_time:=false',
                '-p', 'cmd_vel_timeout:=0.25',
            ], stdout=log, stderr=subprocess.STDOUT)

        # Wait for discovery before testing transitions.
        deadline = time.monotonic() + 10.0
        while status_pub.get_subscription_count() == 0 or not received:
            assert time.monotonic() < deadline, log_path.read_text()
            pump(0.1)

        expect_motion(0.0, 0.0, 0.0, 0.0)  # No goal: hold spawn height.
        first = goal(GoalStatus.STATUS_ACCEPTED)
        send_status(first)
        expect_motion(0.0, 0.0, 0.0, 0.0)  # Accepted is not executing yet.
        first.status = GoalStatus.STATUS_EXECUTING
        send_status(first)
        expect_motion(0.3, 0.1, 0.5, 0.2)

        # Fresh zero commands during an active goal are a pause, not arrival.
        moving_command = command
        command = Twist()
        expect_motion(0.0, 0.0, 0.5, 0.0)
        command = moving_command

        # Losing commands must stop Z as well as XY. Fresh commands can resume.
        publish_command = False
        expect_motion(0.0, 0.0, 0.0, 0.0)
        publish_command = True
        expect_motion(0.3, 0.1, 0.5, 0.2)

        first.status = GoalStatus.STATUS_SUCCEEDED
        send_status(first)
        expect_motion(0.0, 0.0, 0.0, 0.0)
        outdated_first = copy.deepcopy(first)
        outdated_first.status = GoalStatus.STATUS_EXECUTING
        send_status(outdated_first)
        expect_motion(0.0, 0.0, 0.0, 0.0)  # A finished goal cannot restart.

        second = goal(GoalStatus.STATUS_EXECUTING)
        send_status(second, first)  # Older result appears last in the array.
        expect_motion(0.3, 0.1, 0.5, 0.2)
        send_status(first)  # An older snapshot must not stop the current goal.
        expect_motion(0.3, 0.1, 0.5, 0.2)

        # Each terminal outcome must stop movement, despite fresh commands and
        # a downward reading that still requests a climb.
        for terminal in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELING,
                         GoalStatus.STATUS_CANCELED):
            active = goal(GoalStatus.STATUS_EXECUTING)
            send_status(first, active)
            expect_motion(0.3, 0.1, 0.5, 0.2)
            active.status = terminal
            send_status(active, first)
            expect_motion(0.0, 0.0, 0.0, 0.0)

        # Replacement goals must not reuse the previous goal's last command.
        publish_command = False
        active = goal(GoalStatus.STATUS_EXECUTING)
        send_status(active)
        expect_motion(0.0, 0.0, 0.0, 0.0)
        publish_command = True
        expect_motion(0.3, 0.1, 0.5, 0.2)
        send_status()  # An empty status array also means hover.
        expect_motion(0.0, 0.0, 0.0, 0.0)

        # A new goal may share an acceptance timestamp with a finished goal.
        active.status = GoalStatus.STATUS_SUCCEEDED
        replacement = goal(GoalStatus.STATUS_EXECUTING)
        replacement.goal_info.stamp = active.goal_info.stamp
        send_status(replacement, active)
        expect_motion(0.3, 0.1, 0.5, 0.2)
        replacement.status = GoalStatus.STATUS_ABORTED
        active.status = GoalStatus.STATUS_EXECUTING
        send_status(active, replacement)
        expect_motion(0.0, 0.0, 0.0, 0.0)  # Do not revive the superseded goal.
        replacement = goal(GoalStatus.STATUS_EXECUTING)
        send_status(replacement)
        expect_motion(0.3, 0.1, 0.5, 0.2)

        # Status is retained, so it must also work when the follower restarts
        # during an active goal and no new status event is published.
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        with log_path.open('a') as log:
            process = subprocess.Popen([
                os.environ['TERRAIN_FOLLOWING_EXECUTABLE'], '--ros-args',
                '-r', '__ns:=' + namespace,
            ], stdout=log, stderr=subprocess.STDOUT)
        pump(1.0)
        expect_motion(0.3, 0.1, 0.5, 0.2)

        # Server loss must not leave a retained EXECUTING status enabling climb.
        node.destroy_publisher(status_pub)
        pump(2.0)
        expect_motion(0.0, 0.0, 0.0, 0.0)

        # Exploration uses standalone backup goals after navigation fails.
        # Preserve that movement, but do not restart the height correction.
        backup = goal(GoalStatus.STATUS_EXECUTING)
        backup_pub.publish(GoalStatusArray(status_list=[backup]))
        expect_motion(0.3, 0.1, 0.0, 0.2)
        publish_command = False
        expect_motion(0.0, 0.0, 0.0, 0.0)
        publish_command = True
        expect_motion(0.3, 0.1, 0.0, 0.2)
        backup.status = GoalStatus.STATUS_SUCCEEDED
        backup_pub.publish(GoalStatusArray(status_list=[backup]))
        expect_motion(0.0, 0.0, 0.0, 0.0)
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
