import io
import json

from fastapi.testclient import TestClient
from PIL import Image
from mission_control.ros_adapter import UnavailableAdapter


def test_upload_calibration_and_unavailable_ros(monkeypatch, tmp_path, terrain, plan):
    import mission_control.app as module
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", lambda: UnavailableAdapter("ROS unavailable for test"))
    with TestClient(module.app) as client:
        before = client.get("/api/state").json()
        assert before["phase"] == "idle"
        assert not before["ros"]["available"]
        assert client.post("/api/mission/start", json=plan.model_dump(mode="json")).status_code == 422
        assert client.get("/api/state").json()["phase"] == "idle"
        assert client.post("/api/plan/preview", json=plan.model_dump(mode="json")).status_code == 200
        buf = io.BytesIO()
        Image.new("RGB", (32, 16), "green").save(buf, format="PNG")
        files = {"file": ("own-image.png", buf.getvalue(), "image/png")}
        assert client.post("/api/images", files=files).status_code == 422
        registration = {"world_id": "v2_forest", "calibration": {"x_min": -100, "x_max": 100, "y_min": -100, "y_max": 100}}
        uploaded = client.post("/api/images", files=files, data={"registration": json.dumps(registration)})
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
        assert client.post("/api/session/reset").json()["trails"] == {"husky1": [], "parrot1": []}
