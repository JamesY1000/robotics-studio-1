"""Adapted from Nikhil's return-home branch, check both independent spawn poses"""

import importlib.util
import json
from pathlib import Path

from launch import LaunchContext
import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
PLACEMENTS = {
    'husky': {'x': 1.0, 'y': 2.0, 'z': 3.0, 'yaw': 0.0},
    'parrot': {'x': 4.0, 'y': 6.0, 'z': 5.0, 'yaw': 1.0},
    'roi': {'x': 8.0, 'y': 3.0, 'z': 0.0},
    'roi_box': {'x_min': 6.0, 'x_max': 10.0, 'y_min': 1.0, 'y_max': 5.0},
}


@pytest.mark.parametrize('name', ['deploy.launch.py', 'mission_control.launch.py'])
def test_home_reaches_mission_launch(name, tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(name.split('.')[0], LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # Record each include so this needs neither the other packages nor a running stack
    included = {}

    def record(source, launch_arguments):
        included[Path(source).name] = dict(launch_arguments)

    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: '/share/' + package)
    monkeypatch.setattr(module, 'PythonLaunchDescriptionSource', lambda path: path)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', record)

    plan_file = tmp_path / 'plan.json'
    plan_file.write_text(json.dumps({
        'plan': {'world_id': 'world', 'session_id': 'session'}, 'placements': PLACEMENTS}))
    context = LaunchContext()
    context.launch_configurations.update(
        plan_file=str(plan_file), world_instance_id='instance', rviz='False', headless='True',
        log_level='info', stack_log_level='warn')

    module.start(context)

    arguments = included['rs1_mission.launch.py']
    assert arguments['home_from_start'] == 'false'
    assert json.loads(arguments['husky_home']) == [1., 2., 3., 0.]
    assert json.loads(arguments['parrot_home']) == [4., 6., 5., 1.]
    assert (arguments['roi_x'], arguments['roi_y']) == ('8.0', '3.0')
    assert (arguments['roi_x_min'], arguments['roi_y_max']) == ('6.0', '5.0')
    assert arguments['session_id'] == 'session'
