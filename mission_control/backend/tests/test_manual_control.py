import time

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from test_external import Adapter


class ManualAdapter(Adapter):
    def __init__(self):
        super().__init__()
        self.commands = []
        self.stale = False
        self.data["mission"].update(state="manual_assistance", control_mode="husky1", control_revision=1,
                                   manual_input="keyboard", input_revision=0)

    async def call(self, kind):
        await super().call(kind)
        if kind.startswith("input_"):
            self.data["mission"]["manual_input"] = kind.removeprefix("input_")
            self.data["mission"]["input_revision"] += 1
            self.data["mission"]["control_revision"] += 1

    def snapshot(self):
        result = super().snapshot()
        result["mission_at"] = 0 if self.stale else time.monotonic()
        return result

    def publish_manual(self, robot, command=None):
        self.commands.append((robot, command))


@pytest.fixture
def dashboard(monkeypatch, tmp_path, terrain):
    import mission_control.app as module
    monkeypatch.setenv("MISSION_MODE", "external")
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", ManualAdapter)
    with TestClient(module.app) as client:
        yield client, module.app.state.manager.adapter


def test_control_requires_active_mission_fresh_status_and_finished_cleanup(dashboard):
    client, adapter = dashboard
    adapter.data["mission"]["cleanup_complete"] = False
    assert client.post("/api/mission/control/resume").status_code == 409
    assert not adapter.calls
    assert client.post("/api/mission/control/manual_parrot").status_code == 200
    adapter.data["mission"]["cleanup_complete"] = True
    assert client.post("/api/mission/control/resume").status_code == 200
    assert adapter.calls == ["manual_parrot", "resume"]
    adapter.stale = True
    assert client.post("/api/mission/control/manual_husky").status_code == 409
    adapter.stale = False
    adapter.data["mission"]["state"] = "complete"
    assert client.post("/api/mission/control/manual_husky").status_code == 409
    assert client.post("/api/mission/control/anything").status_code == 422


def test_keyboard_cannot_connect_during_autonomy_or_for_other_robot(dashboard):
    client, adapter = dashboard
    for robot, state in (("parrot1", "manual_assistance"), ("husky1", "executing")):
        adapter.data["mission"]["state"] = state
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect(f"/api/teleop/{robot}"):
                pass
    assert not adapter.commands


def test_keyboard_single_owner_disconnect_and_resume_send_zero(dashboard):
    client, adapter = dashboard
    with client.websocket_connect("/api/teleop/husky1") as socket:
        socket.send_json({"enabled": True, "x": 1})
        assert socket.receive_json() == {"ok": True}
        assert adapter.commands[-1][1].x == 1
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("/api/teleop/husky1"):
                pass
        assert adapter.commands[-1][1].enabled
    assert adapter.commands[-1] == ("husky1", None)

    with client.websocket_connect("/api/teleop/husky1") as socket:
        socket.send_json({"enabled": True, "x": 1})
        socket.receive_json()
        assert client.post("/api/mission/control/resume").status_code == 200
        assert adapter.commands[-1] == ("husky1", None)
        with pytest.raises(WebSocketDisconnect):
            socket.receive_json()


@pytest.mark.parametrize("change", ["mode", "revision", "input", "stale", "invalid", "timeout"])
def test_keyboard_revokes_old_input(dashboard, change):
    client, adapter = dashboard
    with client.websocket_connect("/api/teleop/husky1") as socket:
        socket.send_json({"enabled": True, "x": 1})
        socket.receive_json()
        if change == "mode":
            adapter.data["mission"]["control_mode"] = "parrot1"
        elif change == "revision":
            # A switch away and back must also require a new keyboard connection
            adapter.data["mission"]["control_revision"] += 2
        elif change == "input":
            adapter.data["mission"]["manual_input"] = "controller"
        elif change == "stale":
            adapter.stale = True
        if change != "timeout":
            socket.send_json({"enabled": True, "x": 2 if change == "invalid" else 1})
        with pytest.raises(WebSocketDisconnect):
            socket.receive_json()
    assert len(adapter.commands) == 2
    assert adapter.commands[-1] == ("husky1", None)


def test_input_switch_disconnects_keyboard_and_requires_explicit_reconnect(dashboard):
    client, adapter = dashboard
    with client.websocket_connect("/api/teleop/husky1") as socket:
        socket.send_json({"enabled": True, "x": 1})
        socket.receive_json()
        response = client.post("/api/mission/control/input_controller")
        assert response.status_code == 200
        assert response.json()["ros"]["mission"]["manual_input"] == "controller"
        assert adapter.commands[-1] == ("husky1", None)
        with pytest.raises(WebSocketDisconnect):
            socket.receive_json()
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/api/teleop/husky1"):
            pass
    assert client.post("/api/mission/control/input_keyboard").status_code == 200
    with client.websocket_connect("/api/teleop/husky1") as socket:
        socket.send_json({"enabled": False})
        assert socket.receive_json() == {"ok": True}
    adapter.data["mission"]["state"] = "executing"
    assert client.post("/api/mission/control/input_controller").status_code == 409


def test_manual_assistance_remains_active_and_can_be_cancelled(dashboard):
    client, adapter = dashboard
    assert client.get("/api/state").json()["phase"] == "manual_assistance"
    assert client.post("/api/mission/cancel").status_code == 200
    assert adapter.calls == ["cancel"]


def test_managed_assistance_keeps_stack_running(terrain, plan, tmp_path, monkeypatch):
    import asyncio
    from types import SimpleNamespace
    import mission_control.session as session
    from test_session import Adapter as ManagedAdapter, Manager

    # This tests process ownership without depending on Gazebo world generation
    monkeypatch.setattr(terrain, "write_world", lambda *_: None)
    offset = 0
    monkeypatch.setattr(session, "time", SimpleNamespace(
        time=time.time, monotonic=lambda: time.monotonic() + offset))

    async def scenario():
        nonlocal offset
        manager = Manager(terrain, tmp_path, ManagedAdapter)
        try:
            await manager.start(plan)
            manager.adapter.ready(str(plan.session_id))
            await asyncio.sleep(0.3)
            assert manager.adapter.calls == ["start"]
            manager.adapter.data["checks"].update(nav=False, mission_status=True)
            manager.adapter.data["mission"].update(
                state="manual_assistance", cleanup_complete=True, message="Husky needs assistance")
            await asyncio.sleep(0.3)
            offset = 30
            await asyncio.sleep(0.3)
            assert manager.state["phase"] == "manual_assistance"
            assert manager.process is not None and not manager.task.done()
            manager.adapter.data["mission"].update(state="executing", message="Retrying the saved ROI")
            await asyncio.sleep(0.3)
            assert manager.state["phase"] == "executing"
            assert manager.adapter.calls == ["start"]
            await manager.cancel()
            manager.adapter.data["mission"].update(state="cancelled", cleanup_complete=True)
            await asyncio.wait_for(manager.task, 1)
            assert manager.state["phase"] == "cancelled"
            assert manager.adapter.calls == ["start", "cancel"]
        finally:
            await manager.close()

    asyncio.run(scenario())
