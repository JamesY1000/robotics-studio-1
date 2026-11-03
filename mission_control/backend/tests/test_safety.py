import asyncio
import threading
import time
from types import SimpleNamespace as NS

import pytest

from mission_control.ros_adapter import RequestUncertain, RosAdapter
from test_manual_control import dashboard


def test_stop_bypasses_busy_mission_and_preserves_observed_revision(dashboard):
    import mission_control.app as module
    client, adapter = dashboard
    received = []

    async def stop(request):
        received.append(request.model_dump())
        return {"message": "Software stop latched"}

    adapter.set_safety_stop = stop
    manager = module.app.state.manager
    client.portal.call(manager.lock.acquire)
    try:
        response = client.post('/api/safety/stop', json={"robot": "all", "stopped": True})
        assert response.status_code == 200
        response = client.post('/api/safety/stop', json={"robot": "husky1", "stopped": False,
            "expected_instance_id": "observed", "expected_revision": 7})
        assert response.status_code == 200
        assert received[-1]['expected_revision'] == 7
        assert received[-1]['expected_instance_id'] == 'observed'
        assert not adapter.calls  # Neither Reset nor Stop sends Resume
    finally:
        client.portal.call(manager.lock.release)


@pytest.mark.parametrize('payload', [
    {'robot': 'wrong', 'stopped': True}, {'robot': 'all', 'stopped': 'false'},
    {'robot': 'all', 'stopped': False, 'expected_revision': -1},
    {'robot': 'all', 'stopped': False, 'expected_revision': 1.5},
])
def test_stop_rejects_invalid_requests(dashboard, payload):
    client, _ = dashboard
    assert client.post('/api/safety/stop', json=payload).status_code == 422


def test_unknown_stop_outcome_is_not_reported_as_success(dashboard):
    client, adapter = dashboard
    async def stop(_):
        raise RequestUncertain('Stop response timed out; outcome unknown')
    adapter.set_safety_stop = stop
    response = client.post('/api/safety/stop', json={'robot': 'all', 'stopped': True})
    assert response.status_code == 503
    assert 'unknown' in response.json()['detail']


def test_stop_disconnects_keyboard_control(dashboard):
    from starlette.websockets import WebSocketDisconnect
    client, adapter = dashboard
    async def stop(_):
        return {'message': 'Stopped'}
    adapter.set_safety_stop = stop
    with client.websocket_connect('/api/teleop/husky1') as socket:
        assert client.post('/api/safety/stop', json={'robot': 'all', 'stopped': True}).status_code == 200
        with pytest.raises(WebSocketDisconnect):
            socket.receive_json()
    assert adapter.commands[-1] == ('husky1', None)


@pytest.mark.parametrize('change', [{'fresh': False}, {'husky_stopped': True}, {'parrot_stopped': True}])
def test_stop_blocks_resume_and_manual_control(dashboard, change):
    client, adapter = dashboard
    adapter.data['safety'].update(change)
    for operation in ('resume', 'manual_husky', 'manual_parrot', 'input_controller'):
        assert client.post('/api/mission/control/'+operation).status_code == 409
    assert not adapter.calls


def test_adapter_expires_retained_state_and_reset_acknowledgement():
    adapter = RosAdapter.__new__(RosAdapter)
    adapter.lock = threading.Lock()
    adapter.worlds = {}
    adapter.clock_changed_at = time.monotonic()
    adapter.safety_client = NS(service_is_ready=lambda: True)
    adapter.data = {'checks': {}, 'robots': {}, 'mission_at': time.monotonic(),
        'mission': dict(instance_id='coordinator', control_mode='hold', cleanup_complete=True,
                        safety_instance_id='safety', safety_revision=2)}
    adapter.on_safety(NS(instance_id='safety', revision=2, husky_stopped=True, parrot_stopped=True))
    assert adapter.snapshot()['safety']['reset_ready']
    adapter.data['mission']['safety_revision'] = 1
    assert not adapter.snapshot()['safety']['reset_ready']
    adapter.data['safety_at'] -= 1
    assert not adapter.snapshot()['safety']['fresh']


def test_transport_forwards_reset_token_without_changing_it():
    from mission_control.models import SafetyStopRequest
    adapter = RosAdapter.__new__(RosAdapter)
    requests = []
    adapter.safety_type = NS(Request=lambda **fields: NS(**fields))
    def call(request):
        requests.append(request)
        return NS(done=lambda: True, result=lambda: NS(success=True, message='Reset'))
    adapter.safety_client = NS(service_is_ready=lambda: True, call_async=call)
    asyncio.run(adapter.set_safety_stop(SafetyStopRequest(robot='all', stopped=False,
        expected_instance_id='old', expected_revision=9)))
    assert requests[0].expected_instance_id == 'old'
    assert requests[0].expected_revision == 9
