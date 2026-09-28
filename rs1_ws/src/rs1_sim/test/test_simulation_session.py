"""Check the deployment lifetime guard without waiting on simulated time."""
import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "simulation_session.py"
SPEC = importlib.util.spec_from_file_location("simulation_session", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
WorldLease = MODULE.WorldLease


def test_matching_heartbeats_keep_a_paused_world_alive():
    lease = WorldLease("world-a", now=0, timeout=10)
    for wall_time in range(1, 60):
        # No simulation timestamp is needed: a paused world's wall heartbeat
        # still renews its lease.
        lease.observe({"instance_id": "world-a"}, now=wall_time)
        assert lease.check(now=wall_time + 0.5) is None


def test_missing_world_expires_even_without_simulation_time():
    lease = WorldLease("world-a", now=100, timeout=10)
    assert lease.check(now=109) is None
    assert "stopped publishing" in lease.check(now=111)


def test_replaced_or_duplicate_world_irrevocably_stops_deployment():
    lease = WorldLease("world-a", now=0)
    lease.observe({"instance_id": "world-b"}, now=1)
    lease.observe({"instance_id": "world-a"}, now=2)
    assert "replaced" in lease.check(now=2)


def test_malformed_heartbeats_do_not_renew_the_lease():
    lease = WorldLease("world-a", now=0)
    for payload in (None, [], {}, {"instance_id": ""}, {"instance_id": 7}):
        lease.observe(payload, now=9)
    assert "stopped publishing" in lease.check(now=11)
