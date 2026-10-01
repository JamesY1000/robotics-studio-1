from fastapi.testclient import TestClient
import pytest

from mission_control.ros_adapter import RosAdapter, UnavailableAdapter
from mission_control.telemetry_config import connection_settings


def test_configuration_defaults_and_overrides(monkeypatch):
    for suffix in ("VIEWER_URL", "BRIDGE_URL", "VIEWER_PORT", "BRIDGE_PORT"):
        monkeypatch.delenv(f"MISSION_TELEMETRY_{suffix}", raising=False)
    assert connection_settings() == {
        "viewer_url": None, "bridge_url": None, "viewer_port": 8080,
        "bridge_port": 8765, "layout_url": "/api/telemetry/layout",
    }
    monkeypatch.setenv("MISSION_TELEMETRY_VIEWER_URL", "https://robot.example/viewer/")
    monkeypatch.setenv("MISSION_TELEMETRY_BRIDGE_URL", "wss://robot.example/telemetry")
    monkeypatch.setenv("MISSION_TELEMETRY_BRIDGE_PORT", "9001")
    config = connection_settings()
    assert config["bridge_port"] == 9001
    assert config["viewer_url"] == "https://robot.example/viewer/"
    assert config["bridge_url"] == "wss://robot.example/telemetry"


@pytest.mark.parametrize("name,value", [
    ("VIEWER_URL", "javascript:alert(1)"), ("BRIDGE_URL", "http://localhost"),
    ("VIEWER_URL", "https://user:password@example.org"), ("BRIDGE_PORT", "0"),
    ("VIEWER_PORT", "65536"),
])
def test_invalid_configuration_is_rejected(monkeypatch, name, value):
    monkeypatch.setenv(f"MISSION_TELEMETRY_{name}", value)
    with pytest.raises(ValueError):
        connection_settings()


def test_bridge_discovery_checks_exact_namespace():
    class Graph:
        names = [("foxglove_bridge", "/other")]
        def get_node_names_and_namespaces(self):
            return self.names
    adapter = RosAdapter.__new__(RosAdapter)
    adapter.node = Graph()
    assert not adapter.telemetry_available()
    adapter.node.names.append(("foxglove_bridge", "/rs1_telemetry"))
    assert adapter.telemetry_available()


def test_telemetry_configuration_and_layout_work_without_ros(monkeypatch, tmp_path, terrain):
    import mission_control.app as module
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", lambda: UnavailableAdapter("ROS unavailable"))
    monkeypatch.setenv("MISSION_MODE", "external")
    with TestClient(module.app) as client:
        before = client.get("/api/state").json()
        config = client.get("/api/telemetry").json()
        assert config["bridge_available"] is False
        layout = client.get(config["layout_url"], headers={"Origin": "http://localhost:8080"})
        assert layout.status_code == 200
        assert layout.headers["access-control-allow-origin"] == "*"
        assert layout.json()["configById"]
        state = client.get("/api/state", headers={"Origin": "http://localhost:8080"})
        assert "access-control-allow-origin" not in state.headers
        assert state.json() == before
