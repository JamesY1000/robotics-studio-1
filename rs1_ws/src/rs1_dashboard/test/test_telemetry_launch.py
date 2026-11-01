"""Telemetry launches remain independent observers with consistent endpoints."""
import importlib.util
from pathlib import Path
import re

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


@pytest.fixture
def repository(tmp_path):
    for name in (
        "mission_control/frontend/package.json",
        "mission_control/backend/mission_control/app.py",
        "rs1_ws/src/rs1_sim/config/map_presets.json",
        ".venv/lichtblick/1.29.1/index.html",
    ):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    return tmp_path


def prepared_launch(name, repository, monkeypatch, **arguments):
    path = Path(__file__).resolve().parents[1] / "launch" / f"{name}.launch.py"
    spec = importlib.util.spec_from_file_location(f"rs1_dashboard_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in (
        "MISSION_REPO", "MISSION_DATA", "MISSION_TELEMETRY_VIEWER_PORT",
        "MISSION_TELEMETRY_BRIDGE_PORT", "MISSION_TELEMETRY_VIEWER_URL",
        "MISSION_TELEMETRY_BRIDGE_URL",
    ):
        monkeypatch.delenv(name, raising=False)
    if hasattr(module, "get_package_prefix"):
        monkeypatch.setattr(module, "get_package_prefix", lambda _: str(repository / "install"))
    if hasattr(module, "get_package_share_directory"):
        monkeypatch.setattr(module, "get_package_share_directory",
                            lambda package: str(repository / "install/share" / package))
    context = LaunchContext()
    context.launch_configurations.update({"repo_dir": str(repository), **arguments})
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    captured_nodes = []
    if hasattr(module, "Node"):
        original_node = module.Node

        def capture(**kwargs):
            captured_nodes.append(kwargs)
            return original_node(**kwargs)

        monkeypatch.setattr(module, "Node", capture)
    return module, context, captured_nodes


def resolved(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def test_bridge_is_an_observer_in_the_namespace_exempted_by_deployment(repository, monkeypatch):
    module, context, nodes = prepared_launch("telemetry", repository, monkeypatch)
    module.start_telemetry(context)
    assert len(nodes) == 1
    node = nodes[0]
    assert (node["package"], node["executable"]) == ("foxglove_bridge", "foxglove_bridge")
    assert f'/{node["namespace"]}/{node["name"]}' == "/rs1_telemetry/foxglove_bridge"
    parameters = node["parameters"][0]
    assert not {"clientPublish", "services", "parameters", "parametersSubscribe"}.intersection(
        parameters["capabilities"])
    assert "connectionGraph" in parameters["capabilities"]
    # Monitoring must remain responsive while the simulation clock is paused.
    assert parameters["use_sim_time"] is False
    allowed = lambda topic: any(re.fullmatch(pattern, topic) for pattern in parameters["topic_whitelist"])
    for topic in ("/husky1/scan", "/parrot1/map", "/tf", "/tf_static", "/clock", "/mission/status"):
        assert allowed(topic)
    for topic in ("/other_robot/scan", "/husky10/scan", "/unrelated/mission/status"):
        assert not allowed(topic)


def test_explicit_ports_and_addresses_reach_both_services(repository, monkeypatch):
    module, context, nodes = prepared_launch(
        "telemetry", repository, monkeypatch, viewer_host="0.0.0.0", viewer_port="8081",
        bridge_host="127.0.0.1", bridge_port="8766")
    actions = module.start_telemetry(context)
    viewer = next(action for action in actions if type(action) is ExecuteProcess)
    command = [resolved(context, part) for part in viewer.cmd]
    assert command[command.index("--host") + 1] == "0.0.0.0"
    assert command[command.index("--port") + 1] == "8081"
    assert Path(command[command.index("--directory") + 1], "index.html").is_file()
    assert nodes[0]["parameters"][0]["address"] == "127.0.0.1"
    assert nodes[0]["parameters"][0]["port"] == 8766


def test_bridge_can_run_without_viewer_assets(repository, monkeypatch):
    (repository / ".venv/lichtblick/1.29.1/index.html").unlink()
    module, context, nodes = prepared_launch("telemetry", repository, monkeypatch, viewer="false")
    actions = module.start_telemetry(context)
    assert len(nodes) == 1
    assert not any(type(action) is ExecuteProcess for action in actions)


def test_viewer_can_run_without_a_bridge(repository, monkeypatch):
    module, context, nodes = prepared_launch("telemetry", repository, monkeypatch, bridge="false")
    actions = module.start_telemetry(context)
    assert not nodes
    assert len([action for action in actions if type(action) is ExecuteProcess]) == 1


@pytest.mark.parametrize("arguments", [
    {"viewer_port": "0"}, {"bridge_port": "65536"}, {"viewer_port": "invalid"},
    {"viewer_port": "8765", "bridge_port": "8765"},
])
def test_invalid_endpoints_fail_before_starting_processes(repository, monkeypatch, arguments):
    module, context, nodes = prepared_launch("telemetry", repository, monkeypatch, **arguments)
    with pytest.raises(ValueError):
        module.start_telemetry(context)
    assert not nodes


@pytest.mark.parametrize("telemetry", [None, "false", "true"])
def test_dashboard_includes_telemetry_by_default_with_an_explicit_opt_out(repository, monkeypatch, telemetry):
    overrides = {} if telemetry is None else {"telemetry": telemetry}
    module, context, _ = prepared_launch(
        "dashboard", repository, monkeypatch, **overrides,
        viewer_port="8082", bridge_port="8767", telemetry_host="0.0.0.0")
    actions = module.start_dashboard(context)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == (0 if telemetry == "false" else 1)
    if includes:
        arguments = {name: resolved(context, value) for name, value in includes[0].launch_arguments}
        assert arguments == {
            "repo_dir": str(repository), "viewer_port": "8082", "bridge_port": "8767",
            "viewer_host": "0.0.0.0", "bridge_host": "0.0.0.0",
        }
    processes = [action for action in actions if type(action) is ExecuteProcess]
    assert len(processes) == 1
    command = [resolved(context, part) for part in processes[0].cmd]
    assert "mission_control.app:app" in command


@pytest.mark.parametrize("conflicting_service", ["viewer_port", "bridge_port"])
def test_dashboard_rejects_its_own_telemetry_port_collisions(repository, monkeypatch, conflicting_service):
    module, context, _ = prepared_launch(
        "dashboard", repository, monkeypatch, telemetry="true", port="8000",
        **{conflicting_service: "8000"})
    with pytest.raises(ValueError, match="Dashboard, viewer and bridge need different ports"):
        module.start_dashboard(context)


@pytest.mark.parametrize("overrides", [{}, {"show_logs": "false", "log_level": "debug", "stack_log_level": "info"}])
def test_dashboard_passes_logging_defaults_and_overrides(repository, monkeypatch, overrides):
    module, context, _ = prepared_launch("dashboard", repository, monkeypatch, **overrides)
    processes = []
    original_process = module.ExecuteProcess

    def capture(**kwargs):
        processes.append(kwargs)
        return original_process(**kwargs)

    monkeypatch.setattr(module, "ExecuteProcess", capture)
    module.start_dashboard(context)
    environment = processes[0]["additional_env"]
    assert environment["MISSION_SHOW_LOGS"] == overrides.get("show_logs", "true")
    assert environment["MISSION_LOG_LEVEL"] == overrides.get("log_level", "info")
    assert environment["MISSION_STACK_LOG_LEVEL"] == overrides.get("stack_log_level", "warn")
