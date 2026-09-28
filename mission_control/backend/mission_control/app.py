import asyncio
import fcntl
import io
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path
from uuid import uuid4

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image, UnidentifiedImageError

from .models import ExternalStart, Plan, SpawnPlan, UploadCalibration
from .external import ExternalSessionManager
from .deploy import DeploySessionManager
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
    async def sample():
        while True:
            if isinstance(manager, DeploySessionManager):
                await manager.observe_deployment()
            manager.sample_trails()
            await asyncio.sleep(0.5)
    sampler = asyncio.create_task(sample())
    try:
        yield
    finally:
        sampler.cancel()
        await asyncio.gather(sampler, return_exceptions=True)
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
    return await deploy_manager().stop_deployment()


@app.post("/api/mission/cancel")
async def cancel():
    return await app.state.manager.cancel()


@app.post("/api/session/reset")
async def reset():
    return await app.state.manager.reset()


DIST = ROOT / "mission_control/frontend/dist"
if DIST.is_dir():
    app.mount("/", StaticFiles(directory=DIST, html=True), name="dashboard")
