"""Delayed launch actions must not depend on an already-popped argument scope."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


def prepared_launch(name, simulated_time, monkeypatch):
    path = Path(__file__).resolve().parents[1] / "launch" / f"{name}.launch.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update({
        "use_sim_time": str(simulated_time), "husky_x": "12.25", "parrot_x": "13.5",
    })
    captured = []
    node_factory = module.Node

    def capture(**kwargs):
        captured.append(kwargs)
        return node_factory(**kwargs)

    monkeypatch.setattr(module, "Node", capture)
    return module, context, captured


def descendants(actions):
    for action in actions:
        yield action
        if isinstance(action, TimerAction):
            yield from descendants(action.actions)
        elif isinstance(action, GroupAction):
            yield from descendants(action.get_sub_entities())


@pytest.mark.parametrize("simulated_time", [True, False])
def test_delayed_supporting_includes_survive_scope_exit(monkeypatch, simulated_time):
    module, context, nodes = prepared_launch("robot_bringup", simulated_time, monkeypatch)
    actions = module.start_robots(context)
    context.launch_configurations.clear()
    includes = [action for action in descendants(actions) if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 7
    for include in includes:
        # Evaluating these after scope restoration used to raise SubstitutionFailure.
        resolved = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                    for name, value in include.launch_arguments}
        assert resolved["use_sim_time"] == str(simulated_time)
    for node in nodes:
        assert node["parameters"][0]["use_sim_time"] is simulated_time


@pytest.mark.parametrize("robot", ["husky", "parrot"])
@pytest.mark.parametrize("simulated_time", [True, False])
def test_delayed_create_keeps_resolved_coordinates_and_boolean_parameters(monkeypatch, robot, simulated_time):
    module, context, nodes = prepared_launch(f"spawn_{robot}", simulated_time, monkeypatch)
    module.start_robot(context)
    context.launch_configurations.clear()
    create = next(node for node in nodes if node["executable"] == "create")
    assert create["arguments"][create["arguments"].index("-x") + 1] == (
        "12.25" if robot == "husky" else "13.5"
    )
    assert all(isinstance(argument, str) for argument in create["arguments"])
    for node in nodes:
        assert node["parameters"][0]["use_sim_time"] is simulated_time
