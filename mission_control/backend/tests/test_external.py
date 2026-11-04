import asyncio
import copy

import pytest

from mission_control.external import ExternalSessionManager
from mission_control.models import ExternalStart
from mission_control.session import Conflict
from mission_control.ros_adapter import RequestUncertain


class Adapter:
    def __init__(self):
        self.calls = []
        self.closed = False
        self.data = {"available": True, "sim_time": 10,
            "checks": {"clock": True, "mission_status": True, "mission_services": True},
            "robots": {r: {"fresh": True, "x": 90, "y": -48, "stamp": 10} for r in ("husky1", "parrot1")},
            "alignment": {"instance_id": "sim-1", "world_name": "v2_forest",
                "ready": {"husky1": True, "parrot1": True}},
            "mission": {"session_id": "manual", "instance_id": "coordinator-1", "state": "ready",
                "roi": {"x": 88, "y": -48, "z": 0, "frame_id": "world"}, "cleanup_complete": True}}

    def snapshot(self): return copy.deepcopy(self.data)
    def close(self): self.closed = True
    async def configure_roi(self, roi, roi_box=None): self.calls.append(("roi", roi))
    async def call(self, kind): self.calls.append(kind)


def test_external_start_never_launches_and_blocks_duplicate(terrain, plan, tmp_path):
    async def scenario():
        m = ExternalSessionManager(terrain, tmp_path, Adapter)
        for _ in range(3):
            assert m.snapshot()["can_start"]
        assert m.adapter.calls == []
        await m.start(ExternalStart(roi=plan.roi))
        assert m.adapter.calls[0] == ("roi", {**plan.roi.model_dump(), "z": terrain.height(plan.roi.x, plan.roi.y)})
        assert m.adapter.calls[1] == "start"
        with pytest.raises(Conflict): await m.start(ExternalStart())
        assert m.process is None and m.task is None
        assert not (tmp_path / "sessions").exists()
        await m.close()
        assert m.adapter.calls[-1] == "start"  # closing dashboard never cancels ROS
    asyncio.run(scenario())


def test_external_gates_and_supports_existing_roi(terrain, tmp_path):
    async def scenario():
        m = ExternalSessionManager(terrain, tmp_path, Adapter)
        m.adapter.data["alignment"]["ready"]["husky1"] = False
        with pytest.raises(Conflict, match="husky1 alignment"): await m.start(ExternalStart())
        m.adapter.data["alignment"]["ready"]["husky1"] = True
        m.adapter.data["alignment"]["world_name"] = "simple_trees"
        with pytest.raises(Conflict, match="v2_forest"): await m.start(ExternalStart())
        m.adapter.data["alignment"]["world_name"] = "v2_forest"
        await m.start(ExternalStart())
        assert m.adapter.calls == ["start"]  # no ROI mutation if none was requested
        await m.close()
    asyncio.run(scenario())


def test_cancel_return_releases_the_command_for_a_second_cancel(terrain, tmp_path):
    async def scenario():
        adapter = Adapter()
        adapter.data["mission"].update(state="executing", cleanup_complete=False)
        m = ExternalSessionManager(terrain, tmp_path, lambda: adapter)
        m.sample_trails()
        await m.cancel()
        # A status from before Cancel must not end the cancellation early
        m.sample_trails()
        assert m.snapshot()["phase"] == "cancellation_requested"
        adapter.data["mission"].update(state="cancellation_requested", cancel_return=True)
        m.sample_trails()
        assert m.snapshot()["phase"] == "cancellation_requested"
        adapter.data["mission"].update(state="executing", message="Mission cancelled; returning home")
        m.sample_trails()
        assert m.snapshot()["phase"] == "executing"
        assert m.snapshot()["message"] == "Mission cancelled; returning home"
        await m.cancel()
        assert adapter.calls == ["cancel", "cancel"]
        await m.close()

    asyncio.run(scenario())


def test_external_reset_and_restart_are_observation_only(terrain, tmp_path):
    async def scenario():
        adapter = Adapter()
        adapter.data["mission"].update(state="executing", cleanup_complete=False)
        m = ExternalSessionManager(terrain, tmp_path, lambda: adapter)
        m.sample_trails()
        assert len(m.snapshot()["trails"]["husky1"]) == 1
        await m.reset()
        assert m.snapshot()["trails"] == {"husky1": [], "parrot1": []}
        assert m.snapshot()["phase"] == "executing"
        assert adapter.calls == [] and not adapter.closed
        await m.cancel()
        assert adapter.calls == ["cancel"]
        adapter.data["mission"].update(state="cancelled", cleanup_complete=True)
        m.sample_trails()
        assert m.snapshot()["phase"] == "cancelled"
        await m.close()
        recovered = ExternalSessionManager(terrain, tmp_path, lambda: adapter)
        assert recovered.snapshot()["phase"] == "cancelled"
        assert adapter.calls == ["cancel"]
        adapter.data["mission"].update(state="ready", instance_id="coordinator-2")
        recovered.sample_trails()
        assert recovered.snapshot()["can_start"]
        await recovered.close()
    asyncio.run(scenario())


def test_external_new_simulation_clears_old_tf_and_trails(terrain, tmp_path):
    async def scenario():
        m = ExternalSessionManager(terrain, tmp_path, Adapter)
        m.sample_trails()
        previous = m.adapter
        previous.data["alignment"]["instance_id"] = "sim-2"
        m.sample_trails()
        assert previous.closed and m.adapter is not previous
        assert m.snapshot()["trails"] == {"husky1": [], "parrot1": []}
        assert previous.calls == []
        await m.close()
    asyncio.run(scenario())


def test_external_api_rejects_spawn_plans(monkeypatch, tmp_path, terrain, plan):
    from fastapi.testclient import TestClient
    import mission_control.app as module
    monkeypatch.setenv("MISSION_MODE", "external")
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", Adapter)
    with TestClient(module.app) as client:
        assert client.get("/api/state").json()["mode"] == "external"
        assert client.post("/api/mission/start", json=plan.model_dump(mode="json")).status_code == 422
        assert client.post("/api/mission/start", json={"roi": plan.roi.model_dump()}).status_code == 202
        assert client.post("/api/mission/start", json={}).status_code == 409


def test_roi_preview_does_not_start_and_start_rechecks(monkeypatch, tmp_path, terrain, plan):
    from fastapi.testclient import TestClient
    import mission_control.app as module
    monkeypatch.setenv("MISSION_MODE", "external")
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", Adapter)
    point = plan.roi.model_dump()
    with TestClient(module.app) as client:
        preview = client.post("/api/roi/preview", json=point)
        assert preview.status_code == 200
        assert preview.json() == {**point, "z": terrain.height(plan.roi.x, plan.roi.y)}
        assert module.app.state.manager.adapter.calls == []

        # A successful preview is not permission to bypass validation at Start.
        monkeypatch.setattr(terrain, "obstacles", [(plan.roi.x, plan.roi.y)])
        blocked = client.post("/api/roi/preview", json=point)
        assert blocked.status_code == 422
        assert "too close" in blocked.json()["detail"]
        start = client.post("/api/mission/start", json={"roi": point})
        assert start.status_code == 422
        assert start.json()["detail"] == blocked.json()["detail"]
        assert module.app.state.manager.adapter.calls == []


def test_external_definitive_rejection_can_retry_but_timeout_cannot(terrain, tmp_path):
    async def scenario():
        m = ExternalSessionManager(terrain, tmp_path, Adapter)
        async def rejected(_): raise RuntimeError("Action server disappeared")
        m.adapter.call = rejected
        with pytest.raises(RuntimeError): await m.start(ExternalStart())
        assert m.snapshot()["can_start"]
        async def uncertain(_): raise RequestUncertain("Start response timed out")
        m.adapter.call = uncertain
        with pytest.raises(RequestUncertain): await m.start(ExternalStart())
        await m.reset()
        assert not m.snapshot()["can_start"]
        assert "will not repeat" in m.snapshot()["message"]
        m.adapter.data["mission"]["instance_id"] = "coordinator-restarted"
        m.sample_trails()
        assert m.snapshot()["can_start"]
        await m.close()
    asyncio.run(scenario())


def test_external_restart_during_roi_configuration_prevents_start(terrain, plan, tmp_path):
    async def scenario():
        m = ExternalSessionManager(terrain, tmp_path, Adapter)
        m.sample_trails()
        adapter = m.adapter
        async def restart(_, roi_box=None):
            adapter.data["alignment"]["instance_id"] = "sim-new"
            m.sample_trails()  # Must not destroy the client while a command is in flight.
            assert not adapter.closed and m.adapter is adapter
        adapter.configure_roi = restart
        with pytest.raises(Conflict, match="restarted"): await m.start(ExternalStart(roi=plan.roi))
        assert adapter.calls == []
        assert not m.snapshot()["can_start"]
        m.sample_trails()
        assert adapter.closed
        await m.close()
    asyncio.run(scenario())


def test_late_dashboard_uses_initial_verification_and_still_requires_fresh_tf(terrain, tmp_path):
    async def scenario():
        m = ExternalSessionManager(terrain, tmp_path, Adapter)
        alignment = m.adapter.data["alignment"]
        alignment["ready"]["parrot1"] = False
        alignment["initial_verification"] = {"parrot1": {"stamp": 4.35, "xy_m": 0, "yaw_rad": 0}}
        assert m.snapshot()["can_start"]
        assert "differs from Gazebo" in m.snapshot()["message"]
        m.adapter.data["robots"]["parrot1"]["fresh"] = False
        assert not m.snapshot()["can_start"]
        m.adapter.data["robots"]["parrot1"]["fresh"] = True
        alignment["invalid"] = True
        assert not m.snapshot()["can_start"]
        await m.close()
    asyncio.run(scenario())
