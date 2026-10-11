"""Deployment owns robot launch groups, never the independently launched world."""

import asyncio
import copy
import json
import os
import sys
import time

import pytest

from mission_control.deploy import DeploySessionManager
from mission_control.models import ExternalStart, SpawnPlan, XY
from mission_control.session import Conflict


class Adapter:
    def __init__(self):
        self.calls = []
        self.closed = False
        self.foreign = []
        self.data = {
            "available": True,
            "sim_time": 10,
            "world": {
                "instance_id": "world-1",
                "world_name": "v2_forest",
                "ui": True,
                "ign_partition": "deploy-test",
                "session_id": "manual",
                "fresh": True,
            },
            "checks": {
                "clock": True,
                "mission_status": False,
                "mission_services": False,
            },
            "robots": {},
            "alignment": {},
            "mission": {},
        }

    def snapshot(self):
        return copy.deepcopy(self.data)

    def foreign_nodes(self):
        return list(self.foreign)

    def close(self):
        self.closed = True

    async def call(self, kind):
        self.calls.append(kind)

    async def configure_roi(self, roi):
        self.calls.append(("roi", roi))

    def ready(self, session):
        self.data.update(
            checks={
                "clock": True,
                "mission_status": True,
                "mission_services": True,
                "alignment_status": True,
            },
            robots={
                r: {"fresh": True, "x": 90, "y": -48, "stamp": 10}
                for r in ("husky1", "parrot1")
            },
            alignment={
                "instance_id": "alignment-1",
                "world_name": "v2_forest",
                "session_id": str(session),
                "ready": {"husky1": True, "parrot1": True},
            },
            mission={
                "instance_id": "mission-1",
                "session_id": str(session),
                "state": "ready",
                "cleanup_complete": True,
                "roi": {"x": 88, "y": -48, "z": 7, "frame_id": "world"},
            },
        )


class Runtime:
    """A persisted launch registry whose lifetime is independent of the manager."""

    def __init__(self):
        self.record = None
        self.launches = []
        self.stops = []
        self.running = False

    def read(self):
        return copy.deepcopy(self.record)

    def save(self, record):
        self.record = copy.deepcopy(record)

    def alive(self, record):
        return self.running

    def launch(self, record, command, env):
        self.launches.append((copy.deepcopy(record), list(command), dict(env)))
        self.running = True
        record = {
            **record,
            "pid": 10101,
            "process_start": "1234",
            "boot_id": "test-boot",
            "entity_ids": {"husky1": 10, "parrot1": 11},
        }
        self.save(record)
        return copy.deepcopy(record)

    async def stop(self, record):
        self.stops.append(copy.deepcopy(record))
        self.running = False


@pytest.fixture
def spawn_plan(plan, terrain):
    # Deploy keeps the world's existing station at its fixed location, unlike
    # managed mode's fixture which relocates that station beside the start.
    for x, y in ((85, -45), (80, -45), (70, -40), (0, 0)):
        candidate = SpawnPlan(
            **{
                **plan.model_dump(exclude={"roi", "station_offset"}),
                "start": XY(x=x, y=y),
            }
        )
        try:
            terrain.spawn_placements(candidate)
            return candidate
        except ValueError:
            pass
    pytest.fail("No dry clearing separated from the fixed ground station")


def make_manager(terrain, tmp_path, runtime=None, adapter=None, entities=None):
    runtime = runtime or Runtime()
    adapter = adapter or Adapter()
    manager = DeploySessionManager(terrain, tmp_path, lambda: adapter, runtime=runtime)
    entities = set() if entities is None else entities
    removed = []

    async def world_entities(world):
        return set(entities)

    async def remove_entities(world):
        removed.append(world)
        entities.difference_update({"husky1", "parrot1"})

    manager.world_entities = world_entities
    manager.remove_entities = remove_entities
    return manager, runtime, adapter, removed


def test_spawn_is_a_separate_operation_and_cannot_be_duplicated(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        assert len(runtime.launches) == 1
        assert adapter.calls == []
        assert not manager.snapshot()["deployment"]["can_spawn"]
        with pytest.raises(Conflict):
            await manager.spawn(spawn_plan)
        assert len(runtime.launches) == 1
        await manager.close()
        assert adapter.closed
        assert runtime.running and runtime.stops == []

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "change",
    [
        {"ui": False},
        {"fresh": False},
        {"instance_id": ""},
        {"world_name": "simple_trees"},
        {"conflict": True},
    ],
)
def test_spawn_requires_fresh_ui_world_identity(terrain, spawn_plan, tmp_path, change):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        adapter.data["world"].update(change)
        with pytest.raises(Conflict):
            await manager.spawn(spawn_plan)
        assert runtime.launches == []
        await manager.close()

    asyncio.run(scenario())


def test_spawn_requires_running_clock(terrain, spawn_plan, tmp_path):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        adapter.data["checks"]["clock"] = False
        with pytest.raises(Conflict):
            await manager.spawn(spawn_plan)
        assert runtime.launches == []
        await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("existing", ["graph", "scene"])
def test_spawn_rejects_existing_robots_even_without_local_record(
    terrain, spawn_plan, tmp_path, existing
):
    async def scenario():
        entities = {"husky1"} if existing == "scene" else set()
        manager, runtime, adapter, _ = make_manager(
            terrain, tmp_path, entities=entities
        )
        if existing == "graph":
            adapter.foreign = ["/husky1/robot_state_publisher"]
        with pytest.raises(Conflict):
            await manager.spawn(spawn_plan)
        assert runtime.launches == []
        await manager.close()

    asyncio.run(scenario())


def test_spawn_allows_configured_rviz_windows_before_robots(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        adapter.foreign = ["/simulation_session", "/ros_gz_bridge_clock"] + [
            f"/{robot}/{node}"
            for robot in ("husky1", "parrot1")
            for node in (
                "rviz2",
                "rviz_navigation_dialog_action_client",
                "transform_listener_impl_5fbe11c26840",
            )
        ]
        assert manager.snapshot()["deployment"]["can_spawn"]
        await manager.spawn(spawn_plan)
        assert len(runtime.launches) == 1
        assert adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


def test_spawn_allows_dashboard_emergency_stop_node(terrain, spawn_plan, tmp_path):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        adapter.foreign = [
            "/simulation_session",
            "/ros_gz_bridge_clock",
            "/emergency_stop",
        ]
        assert manager.snapshot()["deployment"]["can_spawn"]
        await manager.spawn(spawn_plan)
        assert len(runtime.launches) == 1
        await manager.close()

    asyncio.run(scenario())


def test_spawn_allows_telemetry_but_still_rejects_other_ros_stacks(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        adapter.foreign = [
            "/simulation_session",
            "/ros_gz_bridge_clock",
            "/rs1_telemetry/foxglove_bridge",
        ]
        assert manager.snapshot()["deployment"]["can_spawn"]
        for node in (
            "/husky1/controller_server",
            "/other/foxglove_bridge",
            "/rs1_telemetry/not_the_bridge",
        ):
            adapter.foreign.append(node)
            with pytest.raises(Conflict, match="nodes already exist"):
                await manager.spawn(spawn_plan)
            assert not runtime.launches
            adapter.foreign.pop()
        await manager.spawn(spawn_plan)
        assert len(runtime.launches) == 1
        assert adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


def test_managed_mode_exempts_only_the_dedicated_telemetry_node(terrain, tmp_path):
    from mission_control.session import SessionManager

    adapter = Adapter()
    manager = SessionManager(terrain, tmp_path, lambda: adapter)
    adapter.foreign = ["/rs1_telemetry/foxglove_bridge"]
    assert manager.foreign_nodes() == []
    adapter.foreign.append("/rs1_telemetry/mission_coordinator_node")
    assert manager.foreign_nodes() == ["/rs1_telemetry/mission_coordinator_node"]


@pytest.mark.parametrize(
    "foreign",
    [
        "/husky1/controller_server",
        "/parrot1/robot_state_publisher",
        "/mission_coordinator_node",
        "/other/rviz2",
        "/rviz_navigation_dialog_action_client",
        "/husky1/transform_listener_impl_not_a_generated_id",
    ],
)
def test_rviz_exemption_does_not_hide_other_stacks(
    terrain, spawn_plan, tmp_path, foreign
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        adapter.foreign = ["/husky1/rviz2", "/parrot1/rviz2", foreign]
        with pytest.raises(Conflict, match="nodes already exist"):
            await manager.spawn(spawn_plan)
        assert runtime.launches == []
        await manager.close()

    asyncio.run(scenario())


def test_tf_listener_without_rviz_does_not_get_observer_exemption(terrain, tmp_path):
    manager, _, adapter, _ = make_manager(terrain, tmp_path)
    adapter.foreign = ["/husky1/transform_listener_impl_5fbe11c26840"]
    assert manager.foreign_nodes() == adapter.foreign
    assert not manager.snapshot()["deployment"]["can_spawn"]


def test_ros_graph_preserves_namespaces_for_deployment_checks():
    from mission_control.ros_adapter import RosAdapter

    class Graph:
        def get_node_names_and_namespaces(self):
            return [
                ("simulation_session", "/"),
                ("rviz2", "/husky1"),
                ("controller_server", "/parrot1"),
                ("mission_control_backend", "/"),
                ("_ros2cli_daemon", "/"),
                ("_ros2cli_123", "/"),
            ]

    adapter = RosAdapter.__new__(RosAdapter)
    adapter.node = Graph()
    assert adapter.foreign_nodes() == [
        "/simulation_session",
        "/husky1/rviz2",
        "/parrot1/controller_server",
    ]


def test_spawn_rejects_unregistered_image_calibration(terrain, spawn_plan, tmp_path):
    manager, runtime, _, _ = make_manager(terrain, tmp_path)
    with pytest.raises(ValueError):
        manager.validate_spawn(
            spawn_plan.model_copy(update={"calibration_id": "unregistered"})
        )
    with pytest.raises(ValueError):
        manager.validate_spawn(
            spawn_plan.model_copy(update={"image_id": "missing-image"})
        )
    assert runtime.launches == []


def test_backend_restart_recovers_without_launching_or_starting(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        await manager.close()
        reconnected = Adapter()
        reconnected.ready(spawn_plan.session_id)
        recovered, _, _, _ = make_manager(
            terrain, tmp_path, runtime=runtime, adapter=reconnected
        )
        assert recovered.snapshot()["mode"] == "deploy"
        assert not recovered.snapshot()["deployment"]["can_spawn"]
        assert len(runtime.launches) == 1 and runtime.stops == []
        assert reconnected.calls == []
        with pytest.raises(Conflict):
            await recovered.spawn(spawn_plan)
        await recovered.close()
        assert runtime.running

    asyncio.run(scenario())


def test_concurrent_spawn_requests_launch_once(terrain, spawn_plan, tmp_path):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        results = await asyncio.gather(
            manager.spawn(spawn_plan), manager.spawn(spawn_plan), return_exceptions=True
        )
        assert sum(isinstance(result, Conflict) for result in results) == 1
        assert len(runtime.launches) == 1 and adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


def test_dead_deployment_requires_explicit_cleanup_before_respawn(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, _, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        await manager.close()
        runtime.running = False
        recovered, _, _, _ = make_manager(terrain, tmp_path, runtime=runtime)
        assert not recovered.snapshot()["deployment"]["can_spawn"]
        with pytest.raises(Conflict):
            await recovered.spawn(spawn_plan)
        assert len(runtime.launches) == 1
        await recovered.close()

    asyncio.run(scenario())


def test_roi_is_selected_after_spawn_and_start_is_explicit(
    terrain, plan, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        assert adapter.calls == []
        with pytest.raises(Conflict):
            await manager.start(ExternalStart())
        await manager.start(ExternalStart(roi=plan.roi))
        assert adapter.calls == [
            (
                "roi",
                {**plan.roi.model_dump(), "z": terrain.height(plan.roi.x, plan.roi.y)},
            ),
            "start",
        ]
        assert len(runtime.launches) == 1
        with pytest.raises(Conflict):
            await manager.start(ExternalStart(roi=plan.roi))
        await manager.close()
        assert runtime.stops == []

    asyncio.run(scenario())


@pytest.mark.parametrize("changed", ["world", "alignment", "mission"])
def test_start_requires_the_deployed_world_and_session(
    terrain, plan, spawn_plan, tmp_path, changed
):
    async def scenario():
        manager, _, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        if changed == "world":
            adapter.data["world"]["instance_id"] = "world-restarted"
        else:
            adapter.data[changed]["session_id"] = "foreign-session"
        with pytest.raises(Conflict):
            await manager.start(ExternalStart(roi=plan.roi))
        assert adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


def test_stop_refuses_active_or_uncertain_mission_then_removes_owned_robots(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        entities = {"husky1", "parrot1", "ground_station"}
        manager, runtime, adapter, removed = make_manager(
            terrain, tmp_path, entities=set()
        )
        await manager.spawn(spawn_plan)

        async def world_entities(world):
            return set(entities)

        async def remove_entities(world):
            removed.append(world)
            entities.difference_update({"husky1", "parrot1"})

        manager.world_entities = world_entities
        manager.remove_entities = remove_entities
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        adapter.data["mission"].update(state="executing", cleanup_complete=False)
        with pytest.raises(Conflict):
            await manager.stop_deployment()
        adapter.data["mission"].update(state="ready", cleanup_complete=True)
        manager.command = ("start", 0)
        with pytest.raises(Conflict):
            await manager.stop_deployment()
        assert runtime.stops == [] and removed == []
        manager.command = None
        await manager.stop_deployment()
        assert len(runtime.stops) == 1 and len(removed) == 1
        assert entities == {"ground_station"}
        assert adapter.calls == []
        assert manager.snapshot()["deployment"]["requires_world_restart"]
        assert not manager.snapshot()["deployment"]["can_spawn"]
        await manager.close()

    asyncio.run(scenario())


def test_detached_process_has_durable_ownership_and_survives_runtime_reconnect(
    tmp_path,
):
    from mission_control.deployment_runtime import DeploymentRuntime

    async def scenario():
        runtime = DeploymentRuntime(tmp_path / "deployment.json")
        marker = tmp_path / "executed"
        command = [
            sys.executable,
            "-c",
            (
                "import json, os, pathlib, time; "
                f"record=json.loads(pathlib.Path({str(runtime.path)!r}).read_text()); "
                "assert record['pid'] == os.getpid(); "
                f"pathlib.Path({str(marker)!r}).write_text('owned before exec'); "
                "time.sleep(30)"
            ),
        ]
        record = {
            "phase": "spawning",
            "session_id": "runtime-test",
            "log_path": str(tmp_path / "launch.log"),
        }
        launched = runtime.launch(record, command, dict(os.environ))
        try:
            deadline = time.monotonic() + 5
            while not marker.exists() and time.monotonic() < deadline:
                await asyncio.sleep(0.02)
            assert marker.read_text() == "owned before exec"
            recovered = DeploymentRuntime(runtime.path)
            persisted = recovered.read()
            assert persisted["pid"] == launched["pid"]
            assert recovered.alive(persisted)
            reused_pid = {**persisted, "process_start": "not-this-process"}
            assert not recovered.alive(reused_pid)
            await recovered.stop(reused_pid)
            assert recovered.alive(
                persisted
            )  # stale PID records never signal a process
            await recovered.stop(persisted)
            assert not runtime.alive(launched)
        finally:
            await runtime.stop(launched)

    asyncio.run(scenario())


def test_detached_startup_gate_rejects_unjournaled_process(monkeypatch, tmp_path):
    import mission_control.detached_launch as detached

    path = tmp_path / "deployment.json"
    path.write_text(
        json.dumps(
            {
                "phase": "spawning",
                "session_id": "gate-test",
                "pid": None,
                "command": ["must-not-run"],
            }
        )
    )
    monkeypatch.setattr(
        detached.sys, "argv", ["detached_launch", str(path), "gate-test"]
    )
    monkeypatch.setattr(detached.time, "sleep", lambda _: None)
    attempted = []
    monkeypatch.setattr(detached.os, "execvpe", lambda *args: attempted.append(args))
    with pytest.raises(SystemExit, match="no robots were launched"):
        detached.main()
    assert attempted == []


@pytest.mark.parametrize("mode", ["external", "managed"])
def test_deployment_api_rejects_other_dashboard_modes(
    monkeypatch, tmp_path, terrain, spawn_plan, mode
):
    from fastapi.testclient import TestClient
    import mission_control.app as module

    monkeypatch.setenv("MISSION_MODE", mode)
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", Adapter)
    with TestClient(module.app) as client:
        for route in ("preview", "spawn"):
            assert (
                client.post(
                    f"/api/deployment/{route}", json=spawn_plan.model_dump(mode="json")
                ).status_code
                == 409
            )
        assert client.post("/api/deployment/stop").status_code == 409


def test_deploy_api_spawns_without_start_and_shutdown_leaves_robots(
    monkeypatch, tmp_path, terrain, plan, spawn_plan
):
    from fastapi.testclient import TestClient
    import mission_control.app as module
    import mission_control.deploy as deploy

    adapter, runtime = Adapter(), Runtime()
    monkeypatch.setenv("MISSION_MODE", "deploy")
    monkeypatch.setattr(module, "DATA", tmp_path)
    monkeypatch.setattr(module, "Terrain", lambda _: terrain)
    monkeypatch.setattr(module, "create_adapter", lambda: adapter)
    monkeypatch.setattr(deploy, "DeploymentRuntime", lambda: runtime)

    async def empty_world(self, world):
        return set()

    monkeypatch.setattr(DeploySessionManager, "world_entities", empty_world)
    with TestClient(module.app) as client:
        assert client.get("/api/state").json()["mode"] == "deploy"
        assert (
            client.post(
                "/api/mission/start", json=plan.model_dump(mode="json")
            ).status_code
            == 422
        )
        assert (
            client.post(
                "/api/deployment/spawn", json=plan.model_dump(mode="json")
            ).status_code
            == 422
        )
        response = client.post(
            "/api/deployment/spawn", json=spawn_plan.model_dump(mode="json")
        )
        assert response.status_code == 202, response.text
        assert response.json()["deployment"]["phase"] == "spawning"
        assert (
            client.post(
                "/api/deployment/spawn", json=spawn_plan.model_dump(mode="json")
            ).status_code
            == 409
        )
        assert adapter.calls == []
    assert adapter.closed and runtime.running and runtime.stops == []


def test_world_replaced_during_stop_does_not_remove_replacement_robots(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, adapter, removed = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()

        async def stop_and_replace(record):
            await Runtime.stop(runtime, record)
            adapter.data["world"]["instance_id"] = "replacement-world"

        runtime.stop = stop_and_replace
        await manager.stop_deployment()
        assert len(runtime.stops) == 1
        assert removed == []
        assert runtime.record["phase"] == "stopped"
        await manager.close()

    asyncio.run(scenario())


def test_replacement_world_allows_owned_stack_cleanup_despite_cached_active_mission(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, runtime, adapter, removed = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        adapter.data["mission"].update(state="executing", cleanup_complete=False)
        manager.command = (
            "start",
            0,
        )  # An old request has no acknowledgement after world replacement.
        adapter.data["world"]["instance_id"] = "replacement-world"
        await manager.stop_deployment()
        assert len(runtime.stops) == 1
        assert runtime.stops[0]["world_instance_id"] == "world-1"
        assert removed == [] and adapter.calls == []
        assert runtime.record["phase"] == "stopped"
        assert manager.command is None
        await manager.close()

    asyncio.run(scenario())


def test_second_deployment_requires_a_fresh_restarted_world(
    terrain, spawn_plan, tmp_path
):
    from uuid import uuid4

    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        await manager.stop_deployment()
        following_plan = spawn_plan.model_copy(update={"session_id": uuid4()})
        assert manager.snapshot()["deployment"]["requires_world_restart"]
        with pytest.raises(Conflict, match="Restart rs1_sim"):
            await manager.spawn(following_plan)
        assert len(runtime.launches) == 1
        adapter.data.update(robots={}, alignment={}, mission={})
        adapter.data["world"].update(instance_id="fresh-world", fresh=False)
        assert not manager.snapshot()["deployment"]["can_spawn"]
        with pytest.raises(Conflict):
            await manager.spawn(following_plan)
        adapter.data["world"]["fresh"] = True
        assert not manager.snapshot()["deployment"]["requires_world_restart"]
        assert manager.snapshot()["deployment"]["can_spawn"]
        await manager.spawn(following_plan)
        assert len(runtime.launches) == 2
        assert runtime.record["world_instance_id"] == "fresh-world"
        assert adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


def test_missing_world_heartbeat_does_not_claim_the_world_changed(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, _, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        adapter.data["world"] = {"fresh": False}
        state = manager.snapshot()
        assert state["deployment"]["phase"] == "deployed"
        assert not state["can_start"]
        assert "Waiting for a running world" in state["message"]
        adapter.data["world"] = {
            "instance_id": "confirmed-replacement",
            "world_name": "v2_forest",
            "ui": True,
            "fresh": True,
        }
        state = manager.snapshot()
        assert state["deployment"]["phase"] == "stale"
        assert not state["can_start"]
        await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "observed_state",
    ["executing", "cancellation_requested", "complete", "cancelled", "failed"],
)
def test_reconnect_recovers_selected_roi_from_matching_mission_without_repeating_start(
    terrain, spawn_plan, tmp_path, observed_state
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        assert runtime.record["roi_selected"] is False
        await manager.close()
        # The coordinator accepted Start, but the backend exited before recording
        # that acknowledgement. Its retained ROS status is now authoritative.
        reconnected = Adapter()
        reconnected.ready(spawn_plan.session_id)
        reconnected.data["mission"].update(
            state=observed_state, session_id="foreign-session"
        )
        recovered, _, _, _ = make_manager(
            terrain, tmp_path, runtime=runtime, adapter=reconnected
        )
        recovered.sample_trails()
        assert runtime.record["roi_selected"] is False
        reconnected.data["mission"]["session_id"] = str(spawn_plan.session_id)
        recovered.sample_trails()
        assert runtime.record["roi_selected"] is True
        assert recovered.snapshot()["deployment"]["roi_selected"] is True
        assert reconnected.calls == [] and len(runtime.launches) == 1
        await recovered.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("changed", ["world", "mission"])
def test_cancel_never_calls_a_foreign_mission(terrain, spawn_plan, tmp_path, changed):
    async def scenario():
        manager, _, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        adapter.data["mission"].update(state="executing", cleanup_complete=False)
        if changed == "world":
            adapter.data["world"]["instance_id"] = "foreign-world"
        else:
            adapter.data["mission"]["session_id"] = "foreign-mission"
        with pytest.raises(Conflict):
            await manager.cancel()
        assert adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


def test_cancel_rechecks_owner_after_waiting_for_another_command(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, _, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        adapter.ready(spawn_plan.session_id)
        manager.sample_trails()
        adapter.data["mission"].update(state="executing", cleanup_complete=False)
        await manager.lock.acquire()
        cancellation = asyncio.create_task(manager.cancel())
        await asyncio.sleep(
            0
        )  # The cancellation request now waits for the command lock.
        adapter.data["mission"]["session_id"] = "replacement-mission"
        manager.lock.release()
        with pytest.raises(Conflict):
            await cancellation
        assert adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize("owned", [{}, {"husky1": 999, "parrot1": 11}])
def test_entity_removal_refuses_unrecorded_or_replaced_ids(
    terrain, spawn_plan, tmp_path, owned
):
    async def scenario():
        manager, _, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        manager.deployment["entity_ids"] = owned

        async def models(world):
            return {"husky1": 10, "parrot1": 11}

        attempted = []

        async def request(*args):
            attempted.append(args)
            return "data: true"

        manager.world_models, manager.gazebo_request = models, request
        with pytest.raises(Conflict, match="ownership"):
            await DeploySessionManager.remove_entities(manager, adapter.data["world"])
        assert attempted == []
        await manager.close()

    asyncio.run(scenario())


def test_entity_removal_targets_owned_model_ids_and_preserves_world_assets(
    terrain, spawn_plan, tmp_path
):
    async def scenario():
        manager, _, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        current = {"husky1": 10, "parrot1": 11, "ground_station": 30}

        async def models(world):
            return dict(current)

        async def entities(world):
            return set(current)

        attempted = []

        async def request(world, service, response_type, request_type, body):
            attempted.append((service, body))
            assert service == "remove"
            model = {"id: 10, type: MODEL": "husky1", "id: 11, type: MODEL": "parrot1"}[
                body
            ]
            del current[model]
            return "data: true"

        manager.world_models, manager.world_entities, manager.gazebo_request = (
            models,
            entities,
            request,
        )
        await DeploySessionManager.remove_entities(manager, adapter.data["world"])
        assert attempted == [
            ("remove", "id: 10, type: MODEL"),
            ("remove", "id: 11, type: MODEL"),
        ]
        assert current == {"ground_station": 30}
        await manager.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "failure", [RuntimeError("scene unavailable"), asyncio.TimeoutError()]
)
def test_scene_observation_failure_does_not_stop_later_readiness(
    terrain, spawn_plan, tmp_path, failure
):
    async def scenario():
        manager, runtime, adapter, _ = make_manager(terrain, tmp_path)
        await manager.spawn(spawn_plan)
        manager.deployment.pop("entity_ids")
        runtime.save(manager.deployment)
        adapter.ready(spawn_plan.session_id)

        async def unavailable(world):
            raise failure

        manager.world_models = unavailable
        await manager.observe_deployment()
        manager.sample_trails()
        assert manager.deployment["phase"] == "spawning"
        assert not manager.snapshot()["can_start"]

        async def available(world):
            return {"husky1": 10, "parrot1": 11, "ground_station": 30}

        manager.world_models = available
        await manager.observe_deployment()
        manager.sample_trails()
        assert runtime.record["entity_ids"] == {"husky1": 10, "parrot1": 11}
        assert manager.deployment["phase"] == "deployed"
        assert manager.snapshot()["can_start"]
        assert adapter.calls == []
        await manager.close()

    asyncio.run(scenario())


def test_scene_parser_uses_top_level_model_names_and_ids_only():
    from mission_control.deploy import scene_models

    scene = """name: "v2_forest"
model {
  name: "husky1"
  id: 10
  link {
    name: "parrot1"
    id: 991
  }
}
model {
  name: "parrot1"
  id: 11
  model {
    name: "husky1"
    id: 992
  }
}
model {
  name: "ground_station"
  id: 30
}
light {
  name: "sun"
  id: 40
}
"""
    assert scene_models(scene) == {"husky1": 10, "parrot1": 11, "ground_station": 30}
