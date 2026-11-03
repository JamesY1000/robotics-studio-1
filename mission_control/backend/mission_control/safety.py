"""Shared readiness checks for the software stop"""

import time


def safety_clear(ros):
    state = ros.get("safety", {})
    return bool(state.get("fresh") and state.get("service_ready") and
                state.get("instance_id") and state.get("husky_stopped") is False and
                state.get("parrot_stopped") is False)


def reset_ready(ros):
    state, mission = ros.get("safety", {}), ros.get("mission", {})
    return bool(state.get("fresh") and state.get("service_ready") and
                ros.get("checks", {}).get("mission_status") and mission.get("instance_id") and
                time.monotonic() - ros.get("mission_at", 0) < 0.75 and
                mission.get("control_mode") == "hold" and mission.get("cleanup_complete") and
                mission.get("safety_instance_id") == state.get("instance_id") and
                mission.get("safety_revision") == state.get("revision"))
