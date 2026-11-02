#!/usr/bin/env bash
# Tear down the rs1 forest-sweep stack completely.
#
# Why this matters: `pkill -f 'ros2 launch'` kills the launch process but NOT the
# nodes it spawned (simulation_session.py, robot_state_publisher, slam_toolbox,
# lifecycle_manager, ...).  Those survivors keep their FastRTPS shared-memory
# ports locked, and the NEXT run's nodes then fail with
#   [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portXXXX: open_and_lock_file failed
# which wedges the Nav2 lifecycle and starves the lidar/costmap DDS streams (the
# "observation buffer has not been updated" stall).
set -u

kill_pattern() { pkill -9 -f "$1" 2>/dev/null || true; }

# Simulator (the process is "ign gazebo", NOT "gz sim").
kill_pattern 'ign gazebo'
kill_pattern 'ruby.*ign'

# ros2 launch + every node the launch files spawn.
kill_pattern 'ros2 launch'
kill_pattern 'ros2 run'
kill_pattern 'simulation_session'
kill_pattern 'robot_state_publisher'
kill_pattern 'ros_gz_bridge'
kill_pattern 'parameter_bridge'
kill_pattern 'ekf_node'
kill_pattern 'robot_localization'
kill_pattern 'slam_toolbox'
kill_pattern 'async_slam_toolbox'
kill_pattern 'lifecycle_manager'
kill_pattern 'nav2_'
kill_pattern 'planner_server'
kill_pattern 'controller_server'
kill_pattern 'bt_navigator'
kill_pattern 'behavior_server'
kill_pattern 'smoother_server'
kill_pattern 'waypoint_follower'
kill_pattern 'velocity_smoother'
kill_pattern 'terrain_following'
kill_pattern 'explore'
kill_pattern 'mission_coordinator'
kill_pattern 'parrot_mission'
kill_pattern 'husky_mission'
kill_pattern 'rs1_alignment'
kill_pattern 'mission_alignment'
kill_pattern 'yolo_node'
kill_pattern 'tree_location'
kill_pattern 'test_roi_sampling'

sleep 2

# ros2 daemon (it respawns when killed by pkill).
if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi
kill_pattern 'ros2-daemon'

# FastRTPS shared-memory segments: stale ones from a previous run wedge the next
# run's DDS with "Failed init_port ... open_and_lock_file failed".
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/*_el 2>/dev/null || true

echo "Stack torn down."
