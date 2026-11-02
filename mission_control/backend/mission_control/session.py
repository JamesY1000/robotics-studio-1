"""Single-owner session lifecycle, independent of HTTP/browser connections."""
import asyncio
import copy
import json
import os
import signal
import time
from pathlib import Path

from .models import Calibration, Plan
from .telemetry_config import TELEMETRY_NODE


class Conflict(ValueError):
    pass


class SessionManager:
    def __init__(self, terrain, data_dir: Path, adapter_factory, timeout=180, restore=True):
        self.terrain, self.data_dir, self.adapter_factory = terrain, data_dir, adapter_factory
        self.timeout = timeout
        self.lock = asyncio.Lock()
        self.adapter = adapter_factory()
        self.process = None
        self.task = None
        self.log_file = None
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.state = {"phase": "idle", "message": "Place a shared start and a point ROI to prepare a mission.",
                      "plan": None, "placements": None, "trails": {"husky1": [], "parrot1": []}, "events": []}
        saved = data_dir / "state.json"
        if restore and saved.exists():
            self.state = json.loads(saved.read_text())
            if self.state["phase"] not in ("idle", "complete", "cancelled", "failed"):
                self.transition("failed", "Backend restarted during a session. Reset before launching again; no mission was restarted.")

    def persist(self):
        tmp = self.data_dir / "state.tmp"
        tmp.write_text(json.dumps(self.state, indent=2))
        tmp.replace(self.data_dir / "state.json")

    def transition(self, phase, message):
        self.state.update(phase=phase, message=message)
        self.state["events"].append({"phase": phase, "message": message, "at": time.time()})
        self.state["events"] = self.state["events"][-50:]
        self.persist()

    def image(self, image_id):
        if image_id == "forest-overhead":
            preset = self.terrain.preset
            return {"id": image_id, "calibration_id": preset["calibration_id"], "world_id": preset["world_id"],
                    "name": preset["name"], "calibration": preset["calibration"],
                    "path": str(self.terrain.sim / preset["image"])}
        if not image_id.replace("-", "").isalnum():
            raise ValueError("Invalid image identifier")
        path = self.data_dir / "images" / f"{image_id}.json"
        if not path.exists():
            raise ValueError("Unknown image; upload and calibrate it first")
        return json.loads(path.read_text())

    def images(self):
        return [self.image("forest-overhead")] + [json.loads(p.read_text()) for p in sorted((self.data_dir / "images").glob("*.json"))]

    def validate(self, plan: Plan):
        image = self.image(plan.image_id)
        if plan.world_id != image["world_id"] or plan.calibration_id != image["calibration_id"]:
            raise ValueError("Image calibration does not match this world/plan")
        calibration = Calibration(**image["calibration"])
        placements = self.terrain.placements(plan)
        for name in ("husky", "station", "roi"):
            from .models import XY
            p = placements[name]
            if not calibration.contains(XY(x=p["x"], y=p["y"])):
                raise ValueError(f"{name} is outside the calibrated image")
        return placements

    def snapshot(self):
        result = copy.deepcopy(self.state)
        result["mode"] = "managed"
        result["ros"] = self.adapter.snapshot()
        result["process_running"] = self.process is not None and self.process.returncode is None
        return result

    def foreign_nodes(self):
        # Only our dedicated observer is exempt. Unrelated robot/mission stacks
        # still prevent deployment, even if they use a similar node name.
        return [name for name in self.adapter.foreign_nodes()
                if "/" + name.lstrip("/") != TELEMETRY_NODE]

    def check_cancel_owner(self, ros):
        # External and deploy modes already override this check for their own ROS ownership
        plan = self.state.get("plan") or {}
        if (self.process is None or self.process.returncode is not None or
                ros.get("mission", {}).get("session_id") != plan.get("session_id")):
            raise Conflict("The observed mission does not belong to this session")

    def control_status(self):
        ros = self.adapter.snapshot()
        mission = ros.get("mission", {})
        if (not ros.get("available") or not mission.get("instance_id") or
                time.monotonic() - ros.get("mission_at", 0) >= 1.5):
            raise Conflict("Waiting for fresh mission status")
        self.check_cancel_owner(ros)
        if getattr(self, "command", None):
            raise Conflict("Another mission command is still pending")
        return mission

    async def control(self, operation):
        if operation not in ("manual_husky", "manual_parrot", "resume", "input_keyboard", "input_controller"):
            raise ValueError("Unknown mission control operation")
        async with self.lock:
            mission = self.control_status()
            if operation.startswith("input_"):
                if mission.get("state") != "manual_assistance":
                    raise Conflict("Select manual assistance before choosing an input")
            elif operation == "resume":
                if mission.get("state") != "manual_assistance":
                    raise Conflict("Mission is not in manual assistance")
                if not mission.get("cleanup_complete"):
                    raise Conflict("Previous navigation is still stopping; try Resume again shortly")
            elif mission.get("state") not in ("executing", "manual_assistance"):
                raise Conflict("No active mission to assist")

            # The coordinator rechecks TF and action readiness before retrying the saved ROI
            await self.adapter.call(operation)
            return self.snapshot()

    async def start(self, plan: Plan):
        async with self.lock:
            if self.state["phase"] != "idle" or self.process is not None:
                raise Conflict("A session already exists. Use New session before changing the plan or starting again.")
            placements = self.validate(plan)
            if not self.adapter.snapshot()["available"]:
                raise ValueError(self.adapter.snapshot()["error"])
            foreign = self.foreign_nodes()
            if foreign:
                raise Conflict("ROS domain is already in use: " + ", ".join(sorted(set(foreign))) + ". Stop that stack or select a separate ROS_DOMAIN_ID.")
            directory = self.data_dir / "sessions" / str(plan.session_id)
            if directory.exists():
                raise Conflict("This session identifier was already used; create a new session identifier")
            directory.mkdir(parents=True)
            saved = {"plan": plan.model_dump(mode="json"), "placements": placements}
            (directory / "plan.json").write_text(json.dumps(saved, indent=2))
            self.terrain.write_world(directory / "world.sdf", placements)
            self.state.update(**saved, trails={"husky1": [], "parrot1": []}, events=[])
            self.transition("preparing", "Launching world, robot stacks and shared frame alignment…")
            self.task = asyncio.create_task(self.run(directory))
            return self.snapshot()

    async def launch(self, directory):
        self.log_file = (directory / "ros.log").open("wb")
        spawning = asyncio.create_task(asyncio.create_subprocess_exec(
            "ros2", "launch", "rs1_mission", "mission_control.launch.py",
            f"plan_file:={directory / 'plan.json'}", f"headless:={os.getenv('MISSION_HEADLESS', 'True')}",
            stdout=self.log_file, stderr=asyncio.subprocess.STDOUT, start_new_session=True,
            env={**os.environ, "IGN_PARTITION": f"rs1-mission-{self.state['plan']['session_id']}"}))
        try:
            self.process = await asyncio.shield(spawning)
        except asyncio.CancelledError:
            # Retain ownership even if cancel arrives during subprocess creation.
            self.process = await spawning
            raise

    def ready(self, ros):
        session = self.state["plan"]["session_id"]
        return (ros.get("alignment", {}).get("session_id") == session and
                ros.get("mission", {}).get("session_id") == session and
                all(ros.get("alignment", {}).get("ready", {}).get(r, False) for r in ("husky1", "parrot1")) and
                all(ros.get("robots", {}).get(r, {}).get("fresh", False) for r in ("husky1", "parrot1")) and
                bool(ros.get("checks")) and all(ros["checks"].values()))

    async def run(self, directory):
        try:
            await self.launch(directory)
            deadline = time.monotonic() + self.timeout
            while not self.ready(self.adapter.snapshot()):
                ros = self.adapter.snapshot()
                if not ros.get("checks", {}).get("clock"):
                    message = "Waiting for Gazebo clock and world loading…"
                elif len(ros.get("alignment", {}).get("registered", [])) < 2:
                    count = len(ros.get("alignment", {}).get("registered", []))
                    message = f"Aligning observed robot poses at matching simulation times ({count}/2 registered)…"
                elif not all(ros.get("alignment", {}).get("ready", {}).values()):
                    message = "Verifying initial XY / heading alignment against Gazebo…"
                else:
                    pending = [k for k, v in ros.get("checks", {}).items() if not v]
                    message = "Waiting for navigation readiness: " + ", ".join(pending or ["fresh localization TF"])
                if message != self.state["message"]:
                    self.transition("preparing", message)
                if self.process.returncode is not None:
                    raise RuntimeError("ROS launch exited. Open the session ros.log and resolve the launch error.")
                if time.monotonic() > deadline:
                    ros = self.adapter.snapshot()
                    missing = [k for k, v in ros.get("checks", {}).items() if not v]
                    if not all(ros.get("alignment", {}).get("ready", {}).values()) or not ros.get("alignment"):
                        missing.append("timestamp-matched XY/yaw alignment")
                    raise RuntimeError("Preparation timed out waiting for " + ", ".join(missing or ["fresh robot TF"]) + ". Check ros.log, Gazebo rendering and sensor topics, then reset.")
                await asyncio.sleep(0.25)
            async with self.lock:
                self.transition("ready", "Alignment verified; navigation and mission interfaces ready.")
                await self.adapter.call("start")
                self.transition("executing", "Traversing to the selected ROI.")
            cancellation_deadline = None
            while True:
                ros = self.adapter.snapshot()
                status = ros.get("mission", {})
                if status.get("session_id") == self.state["plan"]["session_id"]:
                    phase = status.get("state")
                    if (ros.get("checks", {}).get("mission_status") and
                            phase in ("executing", "manual_assistance") and
                            self.state["phase"] != "cancellation_requested" and
                            (phase, status.get("message", phase)) != (self.state["phase"], self.state["message"])):
                        self.transition(phase, status.get("message", phase))
                    if phase in ("complete", "cancelled", "failed") and status.get("cleanup_complete"):
                        self.transition(phase, status.get("message", phase))
                        return
                if self.process.returncode is not None:
                    raise RuntimeError("ROS launch stopped during execution. Inspect ros.log and reset.")
                if self.state["phase"] == "cancellation_requested":
                    cancellation_deadline = cancellation_deadline or time.monotonic()+30
                    if time.monotonic() > cancellation_deadline:
                        raise RuntimeError("Cancellation was not acknowledged by both executors within 30 s. Owned simulation stopped; reset required.")
                # Keep the stack available for assistance when navigation or localisation becomes unhealthy
                # The executors report failed attempts and Resume rechecks readiness in the coordinator
                await asyncio.sleep(0.2)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self.stop_process()
            self.transition("failed", str(exc))

    def sample_trails(self):
        # Called by a backend timer, independent of connected browser count.
        if not self.state["plan"] or not self.process:
            return
        for robot, pose in self.adapter.snapshot().get("robots", {}).items():
            if pose.get("fresh") and "x" in pose:
                trail = self.state["trails"][robot]
                if not trail or pose["stamp"] > trail[-1][2]:
                    trail.append([pose["x"], pose["y"], pose["stamp"]])
                    del trail[:-3000]

    async def cancel(self):
        async with self.lock:
            phase = self.state["phase"]
            if phase == "cancellation_requested":
                return self.snapshot()
            if phase not in ("preparing", "ready", "executing", "manual_assistance"):
                raise Conflict("There is no active mission to cancel")
            self.transition("cancellation_requested", "Cancellation requested; waiting for owned work to finish.")
            if phase == "preparing":
                self.task.cancel()
                await asyncio.gather(self.task, return_exceptions=True)
                await self.stop_process()
                self.transition("cancelled", "Preparation cancelled; owned simulation stopped.")
            else:
                try:
                    await self.adapter.call("cancel")
                except Exception as exc:
                    if self.task:
                        self.task.cancel()
                        await asyncio.gather(self.task, return_exceptions=True)
                    await self.stop_process()
                    self.transition("failed", f"Cancellation failed: {exc}. Owned simulation stopped.")
            return self.snapshot()

    async def stop_process(self):
        if self.process:
            # Process group was created by this backend. Never kill by name.
            for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    await asyncio.wait_for(self.process.wait(), timeout)
                    # ros2 launch normally waits for its children on SIGINT.
                    try:
                        os.killpg(self.process.pid, 0)
                    except ProcessLookupError:
                        break
                except asyncio.TimeoutError:
                    pass
            self.process = None
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    async def reset(self):
        async with self.lock:
            if self.task and not self.task.done():
                self.task.cancel()
                await asyncio.gather(self.task, return_exceptions=True)
            await self.stop_process()
            self.adapter.close()
            self.adapter = self.adapter_factory()  # New Context + Buffer, no old statics/trails.
            self.state.update(plan=None, placements=None, trails={"husky1": [], "parrot1": []}, events=[])
            self.transition("idle", "New session. Select a start and ROI.")
            return self.snapshot()

    async def close(self):
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        await self.stop_process()
        self.adapter.close()
        if self.state["phase"] in ("preparing", "ready", "executing", "manual_assistance", "cancellation_requested"):
            self.transition("failed", "Backend shut down; its simulation was stopped. Reset to start a new session.")
        self.persist()
