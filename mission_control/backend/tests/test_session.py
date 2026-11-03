import asyncio
import copy

import pytest
from mission_control.session import Conflict, SessionManager


class Adapter:
    def __init__(self):
        self.calls = []
        self.data = {"available": True, "robots": {}, "checks": {}, "alignment": {}, "mission": {}}
        self.data["safety"] = {"fresh": True, "service_ready": True, "instance_id": "test_safety",
                               "revision": 0, "husky_stopped": False, "parrot_stopped": False}

    def snapshot(self): return copy.deepcopy(self.data)
    def foreign_nodes(self): return []
    def close(self): pass
    async def call(self, kind): self.calls.append(kind)
    def ready(self, session):
        self.data.update(robots={r: {"fresh": True, "x": 10, "y": 20, "stamp": 5} for r in ("husky1", "parrot1")},
            checks={"nav": True, "clock": True}, alignment={"registered": ["husky1", "parrot1"], "session_id": session, "ready": {"husky1": True, "parrot1": True}},
            mission={"session_id": session, "state": "ready"})


class Manager(SessionManager):
    async def launch(self, directory):
        from types import SimpleNamespace
        self.process = SimpleNamespace(returncode=None)
    async def stop_process(self): self.process = None


def test_duplicate_gate_reconnect_cancel_reset(terrain, plan, tmp_path):
    async def scenario():
        m = Manager(terrain, tmp_path, Adapter)
        await m.start(plan)
        await asyncio.sleep(.02)
        assert m.adapter.calls == []  # no alignment/navigation -> no horizontal execution
        with pytest.raises(Conflict): await m.start(plan)
        before = m.state.copy()
        for _ in range(5): m.snapshot()  # reconnect/read-only state cannot mutate lifecycle
        assert m.state == before
        m.adapter.ready("old-session")
        await asyncio.sleep(.3)
        assert m.adapter.calls == []
        m.adapter.ready(str(plan.session_id))
        m.adapter.data["safety"]["husky_stopped"] = True
        await asyncio.sleep(.3)
        assert m.state["phase"] == "preparing" and m.adapter.calls == []
        assert "stop reset" in m.state["message"]
        m.adapter.data["safety"]["husky_stopped"] = False
        await asyncio.sleep(.3)
        assert m.adapter.calls == ["start"]
        m.sample_trails()
        assert len(m.state["trails"]["husky1"]) == 1
        m.adapter.data["checks"]["mission_status"] = True
        for phase in ("sampling", "manual_assistance", "sampling"):
            m.adapter.data["mission"].update(state=phase, message=phase)
            await asyncio.sleep(.3)
            assert m.state["phase"] == phase
        await m.cancel()
        assert m.state["phase"] == "cancellation_requested"
        assert m.adapter.calls == ["start", "cancel"]
        m.adapter.data["mission"].update(state="cancelled", cleanup_complete=True, message="Cancelled")
        await asyncio.sleep(.3)
        assert m.state["phase"] == "cancelled"
        assert m.snapshot()["message"] == "Cancelled"
        await m.reset()
        assert m.state["phase"] == "idle" and m.state["plan"] is None
        assert m.state["trails"] == {"husky1": [], "parrot1": []}
        assert m.adapter.data["alignment"] == {}
        await m.close()
    asyncio.run(scenario())


def test_prepare_cancel_and_timeout(terrain, plan, tmp_path):
    async def scenario():
        m = Manager(terrain, tmp_path, Adapter, timeout=.02)
        await m.start(plan)
        await asyncio.sleep(.3)
        assert m.state["phase"] == "failed"
        assert "timed out" in m.state["message"]
        assert not m.adapter.calls
        await m.reset()
        from uuid import uuid4
        await m.start(plan.model_copy(update={"session_id": uuid4()}))
        await m.cancel()
        assert m.state["phase"] == "cancelled" and m.process is None
        await m.close()
    asyncio.run(scenario())


def test_restart_does_not_launch(terrain, plan, tmp_path):
    async def scenario():
        m = Manager(terrain, tmp_path, Adapter)
        m.state["plan"] = plan.model_dump(mode="json")
        m.transition("executing", "Running")
        recovered = Manager(terrain, tmp_path, Adapter)
        assert recovered.state["phase"] == "failed"
        assert recovered.task is None and recovered.process is None
        await m.close(); await recovered.close()
    asyncio.run(scenario())


def test_image_registration_required(terrain, plan, tmp_path):
    m = Manager(terrain, tmp_path, Adapter)
    with pytest.raises(ValueError, match="does not match"):
        m.validate(plan.model_copy(update={"calibration_id": "unregistered"}))
    with pytest.raises(ValueError, match="Unknown image"):
        m.validate(plan.model_copy(update={"image_id": "uploaded-but-not-calibrated"}))
