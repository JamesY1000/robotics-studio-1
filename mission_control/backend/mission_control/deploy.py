"""Deploy robot stacks into an operator-owned world, surviving dashboard restarts."""

import asyncio
import copy
import json
import os
import re
import time

from .deployment_runtime import DeploymentRuntime
from .external import ExternalSessionManager, TERMINAL
from .models import Calibration, ExternalStart, SpawnPlan
from .session import Conflict


def scene_models(scene):
    """Read direct model name/ID fields from Gazebo's protobuf text response."""
    models = {}
    for block in re.findall(r"(?ms)^model \{\n(.*?)^\}", scene):
        name = re.search(r'^  name: ("(?:\\.|[^"\\])*")$', block, re.M)
        entity_id = re.search(r"^  id: (\d+)$", block, re.M)
        if name and entity_id:
            models[json.loads(name[1])] = int(entity_id[1])
    return models


class DeploySessionManager(ExternalSessionManager):
    def __init__(self, *args, runtime=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.runtime = runtime if runtime is not None else DeploymentRuntime()
        self.deployment = self.runtime.read()

    def world_problem(self, ros):
        world = ros.get("world", {})
        if not ros.get("available"):
            return "ROS adapter is unavailable."
        if (
            not world.get("fresh")
            or not world.get("instance_id")
            or not ros.get("checks", {}).get("clock")
        ):
            return "Waiting for a running world: launch rs1_sim.launch.py ui:=true."
        if world.get("conflict"):
            return "Multiple simulation sessions share this ROS domain. Use a separate domain for each world."
        if not world.get("ui"):
            return "The world was launched with automatic robot spawning. Use external mode or restart with ui:=true."
        if world.get("world_name") != "v2_forest" or world.get("custom_world"):
            return "Deploy currently requires the standard v2_forest world and its registered map."
        return None

    def foreign_nodes(self):
        nodes = {"/" + name.lstrip("/") for name in super().foreign_nodes()}
        allowed = {"/simulation_session", "/ros_gz_bridge_clock", "/emergency_stop"}
        # Combined RViz runs at the root; separate windows use robot namespaces.
        # Only exempt a generated TF listener when RViz is present in that same
        # namespace, so unrelated listeners and robot stacks still block spawn.
        for namespace in ("", "/husky1", "/parrot1"):
            if f"{namespace}/rviz2" not in nodes:
                continue
            allowed.add(f"{namespace}/rviz2")
            # Only the separate robot windows have a Nav2 control panel.
            if namespace:
                allowed.add(f"{namespace}/rviz_navigation_dialog_action_client")
            allowed.update(name for name in nodes if re.fullmatch(
                rf"{namespace}/transform_listener_impl_[0-9a-fA-F]+", name))
        return sorted(nodes - allowed)

    def deployment_view(self, ros):
        record = self.deployment
        world = ros.get("world", {})
        problem = self.world_problem(ros)
        status = "unspawned"
        if record and record.get("phase") != "stopped":
            if world.get("fresh") and record["world_instance_id"] != world.get(
                "instance_id"
            ):
                status, problem = (
                    "stale",
                    "The world changed. Stop the previous robot deployment before spawning again.",
                )
            elif record.get("phase") == "failed" or not self.runtime.alive(record):
                status, problem = (
                    "failed",
                    "Robot deployment stopped or failed. Inspect its ROS log, then Stop robots before trying again.",
                )
            elif record.get("phase") == "spawning":
                status = "spawning"
                if time.time() - record["created_at"] > self.timeout:
                    status, problem = (
                        "failed",
                        "Robot preparation timed out. Inspect its ROS log, then Stop robots before trying again.",
                    )
                else:
                    problem = (
                        problem
                        or "Robots are spawning; waiting for alignment and navigation."
                    )
            else:
                status = "deployed"
        elif record and self.runtime.alive(record):
            status, problem = (
                "failed",
                "The previous robot launch is still running; Stop robots before spawning again.",
            )
        can_spawn = status == "unspawned" and not problem
        requires_restart = bool(
            record
            and record.get("phase") == "stopped"
            and record["world_instance_id"] == world.get("instance_id")
        )
        if requires_restart:
            can_spawn = False
            problem = "Robots stopped. Restart rs1_sim.launch.py ui:=true before deploying again; this Gazebo version cannot safely recreate their camera sensors in the same world."
        if can_spawn:
            foreign = self.foreign_nodes()
            if foreign:
                can_spawn = False
                problem = "ROS robot or mission nodes already exist: " + ", ".join(
                    sorted(set(foreign))
                )
        return {
            "phase": status,
            "can_spawn": can_spawn,
            "requires_world_restart": requires_restart,
            "message": problem
            or (
                "Choose a robot start, then Spawn robots."
                if status == "unspawned"
                else "Robots deployed. Select an ROI and Start Mission."
            ),
            "plan": copy.deepcopy(record["plan"])
            if record and status != "unspawned"
            else None,
            "placements": copy.deepcopy(record["placements"])
            if record and status != "unspawned"
            else None,
            "roi_selected": bool(
                record and status != "unspawned" and record.get("roi_selected")
            ),
            "log_path": record.get("log_path") if record else None,
        }

    def view(self, ros):
        view = super().view(ros)
        deployment = self.deployment_view(ros)
        view.update(mode="deploy", deployment=deployment)
        if deployment["phase"] != "deployed" or self.world_problem(ros):
            view.update(
                can_start=False,
                phase="preparing" if deployment["phase"] == "spawning" else "waiting",
                message=deployment["message"],
            )
        elif (
            ros.get("mission", {}).get("session_id") != self.deployment["session_id"]
            or ros.get("alignment", {}).get("session_id")
            != self.deployment["session_id"]
        ):
            view.update(
                can_start=False,
                phase="waiting",
                message="Waiting for this deployment's mission and alignment nodes.",
            )
        else:
            view["message"] = (
                view["message"]
                .replace("External ROS stack ready.", "Robots ready.")
                .replace(
                    "Restart rs1_mission.launch.py to run another mission.",
                    "Stop robots, restart the world, then deploy again to run another mission.",
                )
            )
            if view["can_start"] and not self.deployment.get("roi_selected"):
                view["message"] = "Robots ready. Select an ROI, then Start Mission."
        return view

    def snapshot(self):
        result = super().snapshot()
        result["process_running"] = bool(
            self.deployment and self.runtime.alive(self.deployment)
        )
        return result

    def sample_trails(self):
        if (
            not self.lock.locked()
            and self.deployment
            and not self.deployment.get("roi_selected")
        ):
            mission = self.adapter.snapshot().get("mission", {})
            if (mission.get("session_id") == self.deployment["session_id"]
                    and mission.get("state") in ("executing", "manual_assistance", "cancellation_requested", *TERMINAL)):
                # Recover a Start acknowledgement lost during a backend restart.
                self.deployment["roi_selected"] = True
                self.runtime.save(self.deployment)
        if (
            not self.lock.locked()
            and self.deployment
            and self.deployment.get("phase") == "spawning"
        ):
            ros = self.adapter.snapshot()
            record = self.deployment
            if (
                not self.world_problem(ros)
                and record["world_instance_id"] == ros["world"]["instance_id"]
                and self.runtime.alive(record)
                and time.time() - record["created_at"] <= self.timeout
                and ros.get("mission", {}).get("session_id") == record["session_id"]
                and ros.get("alignment", {}).get("session_id") == record["session_id"]
                and set(record.get("entity_ids", {})) == {"husky1", "parrot1"}
                and not self.missing(ros)
            ):
                record["phase"] = "deployed"
                self.runtime.save(record)
        super().sample_trails()

    async def observe_deployment(self):
        # Persist Gazebo IDs while this launch is creating its models. Cleanup
        # later addresses those IDs rather than adopting arbitrary matching names.
        async with self.lock:
            record = self.deployment
            if (
                not record
                or record.get("phase") != "spawning"
                or not self.runtime.alive(record)
            ):
                return
            ros = self.adapter.snapshot()
            if (
                self.world_problem(ros)
                or record["world_instance_id"] != ros["world"]["instance_id"]
                or ros.get("mission", {}).get("session_id") != record["session_id"]
            ):
                return
            try:
                models = await self.world_models(ros["world"])
            except (RuntimeError, asyncio.TimeoutError):
                return  # Readiness/timeout stays visible; never guess an entity identity.
            current = self.adapter.snapshot().get("world", {})
            if (
                not current.get("fresh")
                or current.get("instance_id") != record["world_instance_id"]
            ):
                return
            ids = record.setdefault("entity_ids", {})
            changed = False
            for name in ("husky1", "parrot1"):
                if name in models and name not in ids:
                    ids[name] = models[name]
                    changed = True
                elif name in models and ids.get(name) != models[name]:
                    record["phase"] = "failed"
                    changed = True
            if changed:
                self.runtime.save(record)

    def validate_spawn(self, plan: SpawnPlan):
        image = self.image(plan.image_id)
        if (
            image["world_id"] != plan.world_id
            or image["calibration_id"] != plan.calibration_id
        ):
            raise ValueError("Image calibration does not match the selected world")
        if not Calibration(**image["calibration"]).contains(plan.start):
            raise ValueError("Robot start is outside the calibrated image")
        return self.terrain.spawn_placements(plan)

    async def gazebo_request(
        self, world, service, response_type, request_type, request
    ):
        env = {**os.environ, "IGN_PARTITION": world.get("ign_partition", "")}
        # Empty means Gazebo's host/user default; don't force an empty partition.
        if not world.get("ign_partition"):
            env.pop("IGN_PARTITION", None)
        proc = await asyncio.create_subprocess_exec(
            "ign",
            "service",
            "-s",
            f"/world/{world['world_name']}/{service}",
            "--reqtype",
            request_type,
            "--reptype",
            response_type,
            "--timeout",
            "4000",
            "--req",
            request,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        try:
            out, err = await asyncio.wait_for(proc.communicate(), 6)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            proc.kill()
            await proc.wait()
            raise
        if proc.returncode or not out.strip():
            raise RuntimeError(
                f"Gazebo {service} failed: {err.decode(errors='replace')[:300] or 'no response'}"
            )
        return out.decode(errors="replace")

    async def world_entities(self, world):
        return set(await self.world_models(world))

    async def world_models(self, world):
        scene = await self.gazebo_request(
            world, "scene/info", "ignition.msgs.Scene", "ignition.msgs.Empty", ""
        )
        models = scene_models(scene)
        if not models:
            raise RuntimeError(
                "Gazebo scene is unavailable; cannot verify model identities"
            )
        return models

    async def remove_entities(self, world):
        existing = await self.world_models(world)
        owned = self.deployment.get("entity_ids", {})
        for name in ("husky1", "parrot1"):
            if name in existing:
                current = self.adapter.snapshot().get("world", {})
                if (
                    not current.get("fresh")
                    or current.get("conflict")
                    or current.get("instance_id") != world["instance_id"]
                ):
                    raise Conflict(
                        "World changed during cleanup; retry Stop robots after the world is stable"
                    )
                if owned.get(name) != existing[name]:
                    raise Conflict(
                        f"Cannot prove ownership of Gazebo model {name}; restart the world to clear it safely"
                    )
                result = await self.gazebo_request(
                    world,
                    "remove",
                    "ignition.msgs.Boolean",
                    "ignition.msgs.Entity",
                    f"id: {owned[name]}, type: MODEL",
                )
                if not re.search(r"\bdata:\s*true\b", result):
                    raise RuntimeError(f"Gazebo did not acknowledge removal of {name}")
        for _ in range(20):
            if not ({"husky1", "parrot1"} & await self.world_entities(world)):
                return
            await asyncio.sleep(0.1)
        raise RuntimeError(
            "Robots are still present in Gazebo; wait for the world to run before retrying Stop robots"
        )

    async def spawn(self, plan: SpawnPlan):
        async with self.lock:
            ros = self.adapter.snapshot()
            view = self.deployment_view(ros)
            if not view["can_spawn"]:
                raise Conflict(view["message"])
            world = ros["world"]
            placements = self.validate_spawn(plan)
            if {"husky1", "parrot1"} & await self.world_entities(world):
                raise Conflict(
                    "Robot models already exist in this world. Use external mode or restart the world before deploying."
                )
            current = self.adapter.snapshot()
            if (
                self.world_problem(current)
                or current["world"]["instance_id"] != world["instance_id"]
            ):
                raise Conflict(
                    "World changed while checking spawn positions; wait for the current world."
                )
            if self.foreign_nodes():
                raise Conflict(
                    "Robot or mission nodes appeared while preparing deployment; no robots were spawned."
                )
            directory = self.data_dir / "sessions" / str(plan.session_id)
            if directory.exists():
                raise Conflict(
                    "This deployment identifier was already used; reload the dashboard before spawning again"
                )
            directory.mkdir(parents=True)
            saved = {"plan": plan.model_dump(mode="json"), "placements": placements}
            plan_path = directory / "plan.json"
            plan_path.write_text(json.dumps(saved, indent=2))
            record = {**saved, "phase": "spawning", "session_id": str(plan.session_id),
                      "world_instance_id": world["instance_id"], "world_name": world["world_name"],
                      "created_at": time.time(), "roi_selected": False, "log_path": str(directory / "ros.log")}
            command = ["ros2", "launch", "rs1_mission", "deploy.launch.py",
                       f"plan_file:={plan_path}", f"world_instance_id:={world['instance_id']}",
                       f"log_level:={os.getenv('MISSION_LOG_LEVEL', 'info')}",
                       f"stack_log_level:={os.getenv('MISSION_STACK_LOG_LEVEL', 'warn')}"]
            env = {**os.environ}
            if world.get("ign_partition"):
                env["IGN_PARTITION"] = world["ign_partition"]
            else:
                env.pop("IGN_PARTITION", None)
            try:
                self.deployment = self.runtime.launch(record, command, env)
            except Exception:
                self.deployment = self.runtime.read() or {**record, "phase": "failed"}
                raise
            return self.snapshot()

    async def start(self, plan: ExternalStart):
        if not self.deployment or (
            not self.deployment.get("roi_selected") and plan.roi is None
        ):
            raise Conflict(
                "Select an ROI before starting this deployment's first mission"
            )
        await super().start(plan)
        # Atomic parameter update precedes Start; record only after that command is
        # acknowledged. Reconnection also reads the authoritative coordinator ROI.
        self.deployment["roi_selected"] = True
        self.runtime.save(self.deployment)
        return self.snapshot()

    def check_cancel_owner(self, ros):
        if (not self.deployment or ros.get("mission", {}).get("session_id") != self.deployment["session_id"]
                or ros.get("world", {}).get("instance_id") != self.deployment["world_instance_id"]):
            raise Conflict("The observed mission does not belong to this deployment; no command was sent")

    async def stop_deployment(self):
        async with self.lock:
            record = self.deployment
            if not record or record.get("phase") == "stopped":
                return self.snapshot()
            ros = self.adapter.snapshot()
            if not ros.get("world", {}).get("fresh") or ros.get("world", {}).get(
                "conflict"
            ):
                raise Conflict(
                    "Wait for a single world session before cleaning up this deployment; restart the world if it has stopped"
                )
            mission = ros.get("mission", {})
            same_world = ros["world"].get("instance_id") == record["world_instance_id"]
            if same_world and (self.command or mission.get("state") in ("executing", "manual_assistance", "cancellation_requested") or (
                    mission.get("state") in TERMINAL and not mission.get("cleanup_complete"))):
                raise Conflict("Cancel the mission and wait for cleanup before stopping robots")
            if record.get("phase") == "deployed" and self.runtime.alive(record) and same_world and not ros.get("checks", {}).get("mission_status"):
                raise Conflict("Mission status is unavailable; restore its connection before stopping deployed robots")
            await self.runtime.stop(record)
            # Shutdown may take seconds. A replacement world must never inherit
            # the old decision to remove models, even if its model names match.
            current = self.adapter.snapshot().get("world", {})
            if not current.get("fresh") or current.get("conflict"):
                raise Conflict(
                    "Robot nodes stopped. Wait for the world connection, then retry Stop robots to finish cleanup"
                )
            if current.get("instance_id") == record["world_instance_id"]:
                await self.remove_entities(current)
            record["phase"] = "stopped"
            self.runtime.save(record)
            self.adapter.close()
            self.adapter = self.adapter_factory()
            self.alignment_instance = self.mission_instance = self.last_sim_time = None
            self.command = self.command_error = None
            self.state.update(trails={"husky1": [], "parrot1": []}, events=[])
            return self.snapshot()
