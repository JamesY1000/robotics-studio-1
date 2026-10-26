import io
import json

from fastapi.testclient import TestClient
from PIL import Image
from mission_control.ros_adapter import UnavailableAdapter


def test_upload_calibration_and_unavailable_ros(monkeypatch, tmp_path, terrain, plan):
    import mission_control.app as module

    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(
        module, "create_adapter", lambda: UnavailableAdapter("ROS unavailable for test")
    )
    with TestClient(module.app) as client:
        before = client.get("/api/state").json()
        assert before["phase"] == "idle"
        assert not before["ros"]["available"]
        assert (
            client.post(
                "/api/mission/start", json=plan.model_dump(mode="json")
            ).status_code
            == 422
        )
        assert client.get("/api/state").json()["phase"] == "idle"
        assert (
            client.post(
                "/api/plan/preview", json=plan.model_dump(mode="json")
            ).status_code
            == 200
        )
        buf = io.BytesIO()
        Image.new("RGB", (32, 16), "green").save(buf, format="PNG")
        files = {"file": ("own-image.png", buf.getvalue(), "image/png")}
        assert client.post("/api/images", files=files).status_code == 422
        registration = {
            "world_id": "v2_forest",
            "calibration": {"x_min": -100, "x_max": 100, "y_min": -100, "y_max": 100},
        }
        uploaded = client.post(
            "/api/images", files=files, data={"registration": json.dumps(registration)}
        )
        assert uploaded.status_code == 200
        image = uploaded.json()
        assert image["calibration_id"] != "forest-edges-v1"
        assert "path" not in image
        p = plan.model_dump(mode="json")
        p["image_id"] = image["id"]
        assert client.post("/api/plan/preview", json=p).status_code == 422
        p["calibration_id"] = image["calibration_id"]
        assert client.post("/api/plan/preview", json=p).status_code == 200
        assert client.get(image["url"]).status_code == 200
        assert len(client.get("/api/presets").json()) == 2


def test_safety_stop_api_controls_each_robot_and_validates_target(
    monkeypatch, tmp_path, terrain
):
    import mission_control.app as module

    class InterlockAdapter(UnavailableAdapter):
        def __init__(self):
            super().__init__("test adapter")
            self.stops = {"husky1": False, "parrot1": False}
            self.calls = []

        def snapshot(self):
            return {
                "available": True,
                "robots": {},
                "checks": {},
                "safety": {
                    "available": True,
                    "service_ready": True,
                    "husky1_stopped": self.stops["husky1"],
                    "parrot1_stopped": self.stops["parrot1"],
                },
            }

        async def set_safety_stop(self, robot, stopped):
            self.calls.append((robot, stopped))
            for name in self.stops if robot == "all" else [robot]:
                self.stops[name] = stopped

    adapter = InterlockAdapter()
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", lambda: adapter)
    with TestClient(module.app) as client:
        response = client.post(
            "/api/safety/stop", json={"robot": "husky1", "stopped": True}
        )
        assert response.status_code == 200
        assert response.json()["ros"]["safety"]["husky1_stopped"] is True
        assert response.json()["ros"]["safety"]["parrot1_stopped"] is False
        reset = client.post(
            "/api/safety/stop", json={"robot": "husky1", "stopped": False}
        )
        assert reset.status_code == 200
        assert adapter.calls == [("husky1", True), ("husky1", False)]
        both = client.post("/api/safety/stop", json={"robot": "all", "stopped": True})
        assert both.status_code == 200
        assert both.json()["ros"]["safety"]["husky1_stopped"] is True
        assert both.json()["ros"]["safety"]["parrot1_stopped"] is True
        assert adapter.calls[-1] == ("all", True)
        assert (
            client.post(
                "/api/safety/stop", json={"robot": "both", "stopped": True}
            ).status_code
            == 422
        )
        assert client.post("/api/session/reset").json()["trails"] == {
            "husky1": [],
            "parrot1": [],
        }
