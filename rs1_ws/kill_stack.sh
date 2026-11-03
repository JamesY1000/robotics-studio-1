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

# Usage:
#   ./kill_stack.sh           Tear the stack down (SIGTERM first, then SIGKILL).
#   ./kill_stack.sh --check    Non-destructive pre-flight: is the machine clean?

# Everything the launch files can start.  One list drives teardown, verification
# and the pre-flight check.
PATTERNS=(
    'ign gazebo' 'ruby.*ign'
    'ros2 launch' 'ros2 run'
    'simulation_session' 'robot_state_publisher'
    'ros_gz_bridge' 'parameter_bridge'
    'ekf_node' 'robot_localization'
    'slam_toolbox' 'async_slam_toolbox'
    'lifecycle_manager' 'nav2_'
    'planner_server' 'controller_server' 'bt_navigator'
    'behavior_server' 'smoother_server' 'waypoint_follower' 'velocity_smoother'
    'terrain_following' 'explore' 'map_merge'
    'mission_coordinator' 'parrot_mission' 'husky_mission'
    'rs1_alignment' 'mission_alignment'
    'yolo_node' 'tree_location' 'test_roi_sampling'
    'rviz2'
)

count_matching() {
    local total=0 p n
    for p in "${PATTERNS[@]}"; do
        n=$(pgrep -f "$p" 2>/dev/null | wc -l)
        total=$((total + n))
    done
    printf '%s' "$total"
}

list_matching() {
    local p
    for p in "${PATTERNS[@]}"; do
        pgrep -af "$p" 2>/dev/null | sed 's/^/  /' || true
    done
}

# Non-destructive pre-flight: run this before launching the terminals.
if [[ "${1:-}" == "--check" ]]; then
    sims=$(pgrep -f 'ign gazebo' 2>/dev/null | wc -l)
    leftovers=$(count_matching)
    echo "Pre-flight check"
    echo "  ign gazebo instances : $sims   (must be 0 before launch)"
    echo "  matching processes   : $leftovers"
    if [[ "$sims" -ne 0 || "$leftovers" -ne 0 ]]; then
        echo "  -> NOT clean; run ./kill_stack.sh first"
        exit 1
    fi
    echo "  clean."
    exit 0
fi

# 1) Graceful first (SIGTERM): nodes that exit cleanly release their FastRTPS
#    shared-memory ports.  Hard kills leave the port files that wedge the next
#    run ("Failed init_port ... open_and_lock_file failed").
for p in "${PATTERNS[@]}"; do pkill -TERM -f "$p" 2>/dev/null || true; done

# Give them a moment to exit on their own.
for _ in $(seq 1 16); do
    [[ "$(count_matching)" -eq 0 ]] && break
    sleep 0.5
done

# 2) Force anything still alive.
for p in "${PATTERNS[@]}"; do pkill -9 -f "$p" 2>/dev/null || true; done
sleep 1

# 3) ros2 daemon (it respawns when killed by pkill).
if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi
pkill -9 -f 'ros2-daemon' 2>/dev/null || true

# 4) FastRTPS shared-memory files: stale ones from a previous run wedge the next
#    run's DDS with "Failed init_port ... open_and_lock_file failed", which stalls
#    the EKF and the lidar/costmap streams ("observation buffer has not been
#    updated").
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/*_el 2>/dev/null || true

# 5) Verify nothing survived.
remaining=$(count_matching)
if [[ "$remaining" -ne 0 ]]; then
    echo "WARNING: $remaining process(es) still match after teardown:"
    list_matching
    exit 1
fi
echo "Stack torn down."
