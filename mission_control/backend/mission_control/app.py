import asyncio
import fcntl
import io
import json
import os
from contextlib import asynccontextmanager, suppress
from pathlib import Path
from typing import Literal
from uuid import uuid4

from fastapi import FastAPI, File, Form, HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image, UnidentifiedImageError

from .models import ExternalStart, Plan, SpawnPlan, TeleopCommand, UploadCalibration, XY
from .external import ExternalSessionManager
from .deploy import DeploySessionManager
from .deployment_logs import DeploymentLogTail, stream_deployment_logs
from .ros_adapter import create_adapter
from .session import Conflict, SessionManager
from .terrain import Terrain
from .telemetry_config import connection_settings

ROOT = Path(os.getenv("MISSION_REPO", Path(__file__).resolve().parents[3]))
DATA = Path(os.getenv("MISSION_DATA", ROOT / ".mission-control"))
os.environ.setdefault("ROS_DOMAIN_ID", "42")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")


@asynccontextmanager
async def lifespan(app):
    # One owner per ROS domain, even if a second backend uses another data directory.
    lock = open(f"/tmp/rs1-mission-control-{os.environ['ROS_DOMAIN_ID']}.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock.close()
        raise RuntimeError("A mission-control backend already owns this ROS domain") from exc
    mode = os.getenv("MISSION_MODE", "managed")
    managers = {"managed": SessionManager, "external": ExternalSessionManager, "deploy": DeploySessionManager}
    if mode not in managers:
        raise ValueError("MISSION_MODE must be managed, external or deploy")
    manager_type = managers[mode]
    manager = manager_type(Terrain(ROOT / "rs1_ws/src/rs1_sim"), DATA, create_adapter,
                             timeout=float(os.getenv("MISSION_PREPARE_TIMEOUT", "180")))
    app.state.manager = manager
    app.state.teleop = None
    async def sample():
        while True:
            if isinstance(manager, DeploySessionManager):
                await manager.observe_deployment()
            manager.sample_trails()
            await asyncio.sleep(0.5)
    sampler = asyncio.create_task(sample())
    tasks = [sampler]
    if isinstance(manager, DeploySessionManager) and os.getenv("MISSION_SHOW_LOGS", "true").lower() in ("true", "1"):
        tail = DeploymentLogTail(manager.deployment)
        tasks.append(asyncio.create_task(stream_deployment_logs(manager, tail)))
    try:
        yield
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await stop_teleop()
        await manager.close()
        lock.close()


app = FastAPI(title="RS1 Mission Control", lifespan=lifespan)


@app.exception_handler(ValueError)
async def validation_error(_, exc):
    from fastapi.responses import JSONResponse
    return JSONResponse(status_code=409 if isinstance(exc, Conflict) else 422,
                        content={"detail": str(exc)})


@app.exception_handler(RuntimeError)
async def ros_error(_, exc):
    from fastapi.responses import JSONResponse
    return JSONResponse(status_code=503, content={"detail": str(exc)})


@app.get("/api/state")
async def state():
    return app.state.manager.snapshot()


@app.get("/api/telemetry")
def telemetry():
    adapter = app.state.manager.adapter
    return {**connection_settings(),
            "bridge_available": getattr(adapter, "telemetry_available", lambda: False)()}


@app.get("/api/telemetry/layout")
def telemetry_layout():
    # This public, static project layout is fetched by the viewer on another port.
    # Keep CORS limited to this route rather than exposing mission-control APIs.
    return FileResponse(ROOT / "mission_control/telemetry/default-layout.json",
                        media_type="application/json",
                        headers={"Access-Control-Allow-Origin": "*"})


@app.get("/api/presets")
def presets():
    return [{**{k: v for k, v in item.items() if k != "path"}, "url": f"/api/images/{item['id']}"}
            for item in app.state.manager.images()]


@app.get("/api/images/{image_id}")
def image(image_id: str):
    return FileResponse(app.state.manager.image(image_id)["path"])


@app.post("/api/images")
async def upload(file: UploadFile = File(...), registration: str = Form(...)):
    metadata = UploadCalibration.model_validate_json(registration)
    raw = await file.read(10 * 1024 * 1024 + 1)
    if len(raw) > 10 * 1024 * 1024:
        raise HTTPException(413, "Upload limit is 10 MiB")
    try:
        image = Image.open(io.BytesIO(raw))
        if image.format not in ("PNG", "JPEG"):
            raise ValueError("Upload a PNG or JPEG image")
        if image.width * image.height > 16_000_000:
            raise ValueError("Image limit is 16 megapixels")
        image.load()
        image = image.convert("RGB")
    except (UnidentifiedImageError, OSError, Image.DecompressionBombError) as exc:
        raise ValueError("Upload a valid PNG or JPEG image") from exc
    identifier = str(uuid4())
    directory = DATA / "images"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{identifier}.png"
    image.save(path)
    item = {"id": identifier, "calibration_id": identifier, "world_id": metadata.world_id,
            "calibration": metadata.calibration.model_dump(), "name": (file.filename or "Uploaded image")[:100],
            "path": str(path)}
    (directory / f"{identifier}.json").write_text(json.dumps(item))
    return {**{k: v for k, v in item.items() if k != "path"}, "url": f"/api/images/{identifier}"}


@app.post("/api/plan/preview")
def preview(plan: Plan):
    return app.state.manager.validate(plan)


@app.post("/api/roi/preview")
def roi_preview(point: XY):
    # Preview the same terrain checks used at mission start without configuring
    # the coordinator or starting a mission.
    terrain = app.state.manager.terrain
    terrain.check_point("ROI", point, 0.5)
    return {**point.model_dump(), "z": terrain.height(point.x, point.y)}


@app.post("/api/mission/start", status_code=202)
async def start(plan: Plan | ExternalStart):
    manager = app.state.manager
    expected = ExternalStart if isinstance(manager, ExternalSessionManager) else Plan
    if not isinstance(plan, expected):
        raise ValueError("External/deploy modes accept only an ROI; managed mode requires a complete launch plan")
    return await manager.start(plan)


def deploy_manager():
    manager = app.state.manager
    if not isinstance(manager, DeploySessionManager):
        raise Conflict("Robot deployment requires mode:=deploy")
    return manager


@app.post("/api/deployment/preview")
def deployment_preview(plan: SpawnPlan):
    return deploy_manager().validate_spawn(plan)


@app.post("/api/deployment/spawn", status_code=202)
async def deployment_spawn(plan: SpawnPlan):
    return await deploy_manager().spawn(plan)


@app.post("/api/deployment/stop")
async def deployment_stop():
    await stop_teleop()
    return await deploy_manager().stop_deployment()


@app.post("/api/mission/cancel")
async def cancel():
    await stop_teleop()
    return await app.state.manager.cancel()


@app.post("/api/session/reset")
async def reset():
    await stop_teleop()
    return await app.state.manager.reset()


async def stop_teleop(code=1000, reason="Keyboard control stopped"):
    record = app.state.teleop
    if record is None:
        return

    # Revoke ownership before awaiting shutdown so an old socket cannot send another command
    app.state.teleop = None
    with suppress(Exception):
        if app.state.manager.adapter is record["adapter"]:
            record["adapter"].publish_manual(record["robot"])
    with suppress(Exception):
        await record["socket"].close(code=code, reason=reason)


def check_manual_control(manager, adapter, robot):
    if manager.adapter is not adapter or manager.lock.locked():
        raise Conflict("Mission control is changing; connect keyboard again shortly")
    mission = manager.control_status()
    if mission.get("state") != "manual_assistance" or mission.get("control_mode") != robot:
        raise Conflict("This robot is not selected for manual control")
    if mission.get("manual_input") != "keyboard":
        raise Conflict("Select Keyboard input before connecting keyboard control")
    return mission["instance_id"], mission.get("control_revision", 0)


@app.post("/api/mission/control/{operation}")
async def mission_control(operation: Literal["manual_husky", "manual_parrot", "resume", "input_keyboard", "input_controller"]):
    await stop_teleop()
    return await app.state.manager.control(operation)


@app.websocket("/api/teleop/{robot}")
async def teleop(socket: WebSocket, robot: str):
    record = None
    try:
        if robot not in ("husky1", "parrot1"):
            raise Conflict("Unknown robot")
        if app.state.teleop is not None:
            raise Conflict("Keyboard control is already connected in another panel")

        manager = app.state.manager
        adapter = manager.adapter
        identity = check_manual_control(manager, adapter, robot)
        record = {"socket": socket, "adapter": adapter, "robot": robot}
        app.state.teleop = record
        await socket.accept()

        while app.state.teleop is record:
            raw = await asyncio.wait_for(socket.receive_json(), timeout=0.35)
            if app.state.teleop is not record:
                break
            if check_manual_control(manager, adapter, robot) != identity:
                raise Conflict("Mission control changed; reconnect and press Space again")

            # Only publish newly received input - never repeat a cached nonzero command here
            command = TeleopCommand.model_validate(raw)
            adapter.publish_manual(robot, command)
            await socket.send_json({"ok": True})
    except WebSocketDisconnect:
        pass
    except (asyncio.TimeoutError, ValueError, RuntimeError) as exc:
        reason = "Keyboard input timed out" if isinstance(exc, asyncio.TimeoutError) else str(exc)
        reason = reason.encode("utf-8")[:120].decode("utf-8", errors="ignore")
        if record is not None and app.state.teleop is record:
            await stop_teleop(code=1008, reason=reason)
        else:
            with suppress(Exception):
                await socket.close(code=1008, reason=reason)
    finally:
        # A retiring connection must not stop a newer operator's input
        if record is not None and app.state.teleop is record:
            await stop_teleop()
        else:
            with suppress(Exception):
                await socket.close()


DIST = ROOT / "mission_control/frontend/dist"
if DIST.is_dir():
    app.mount("/", StaticFiles(directory=DIST, html=True), name="dashboard")
