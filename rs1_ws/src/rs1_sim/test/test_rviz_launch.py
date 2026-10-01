"""UI displays must not enable robot bringup or change normal launch defaults."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def selected_includes(actions, context):
    for action in actions:
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        if isinstance(action, GroupAction):
            yield from selected_includes(action.get_sub_entities(), context)
        elif isinstance(action, IncludeLaunchDescription):
            yield Path(action.launch_description_source.location).name


@pytest.mark.parametrize("arguments, expected_rviz, expected_stack", [
    ({}, True, "robot_bringup.launch.py"),
    ({"rviz": "false"}, False, "robot_bringup.launch.py"),
    ({"ui": "true"}, False, None),
    ({"ui": "true", "rviz": "true"}, True, "rviz.launch.py"),
    ({"ui": "True", "rviz": "false"}, False, None),
    ({"ui": "false", "rviz": "true"}, True, "robot_bringup.launch.py"),
])
def test_world_and_display_selection(arguments, expected_rviz, expected_stack):
    path = Path(__file__).resolve().parents[1] / "launch" / "rs1_sim.launch.py"
    spec = importlib.util.spec_from_file_location("rs1_sim", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert IfCondition(LaunchConfiguration("rviz")).evaluate(context) is expected_rviz
    # Resolve include paths, but do not execute any process or nested launch.
    for action in description.entities:
        if isinstance(action, GroupAction):
            for child in action.get_sub_entities():
                if isinstance(child, IncludeLaunchDescription):
                    child.launch_description_source.get_launch_description(context)
    expected = ["gz_world.launch.py"] + ([expected_stack] if expected_stack else [])
    assert list(selected_includes(description.entities, context)) == expected
