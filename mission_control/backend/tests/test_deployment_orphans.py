"""A ROS launch leader can die while its process group remains alive."""
import asyncio
import os
import sys
import time

import pytest

from mission_control.deployment_runtime import DeploymentRuntime, process_identity
from mission_control.session import Conflict


def test_stop_recovers_and_terminates_children_after_launch_leader_exits(tmp_path):
    async def scenario():
        runtime = DeploymentRuntime(tmp_path / "deployment.json")
        marker = tmp_path / "child.pid"
        command = [sys.executable, "-c", (
            "import os, pathlib, subprocess, sys; "
            "child=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)']); "
            f"pathlib.Path({str(marker)!r}).write_text(str(child.pid)); "
            "os._exit(0)"
        )]
        record = runtime.launch(
            {"phase": "spawning", "session_id": "orphan-test", "log_path": str(tmp_path / "launch.log")},
            command, dict(os.environ),
        )
        try:
            deadline = time.monotonic() + 5
            while (not marker.exists() or process_identity(record["pid"])) and time.monotonic() < deadline:
                await asyncio.sleep(0.02)
            child_pid = int(marker.read_text())
            assert process_identity(record["pid"]) is None
            assert process_identity(child_pid)
            assert os.getpgid(child_pid) == record["pid"]
            assert os.getsid(child_pid) == record["pid"]
            recovered = DeploymentRuntime(runtime.path)
            persisted = recovered.read()
            assert recovered.alive(persisted)
            await recovered.stop(persisted)
            assert not recovered.alive(persisted)
            assert process_identity(child_pid) is None
        finally:
            await runtime.stop(record)
    asyncio.run(scenario())


def test_orphan_group_with_wrong_token_is_never_signalled(tmp_path):
    async def scenario():
        runtime = DeploymentRuntime(tmp_path / "deployment.json")
        marker = tmp_path / "child.pid"
        command = [sys.executable, "-c", (
            "import os, pathlib, subprocess, sys; "
            "child=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)']); "
            f"pathlib.Path({str(marker)!r}).write_text(str(child.pid)); "
            "os._exit(0)"
        )]
        record = runtime.launch(
            {"phase": "spawning", "session_id": "orphan-identity-test", "log_path": str(tmp_path / "launch.log")},
            command, dict(os.environ),
        )
        try:
            deadline = time.monotonic() + 5
            while (not marker.exists() or process_identity(record["pid"])) and time.monotonic() < deadline:
                await asyncio.sleep(0.02)
            child_pid = int(marker.read_text())
            wrong_token = {**record, "ownership_token": "another-deployment"}
            assert not runtime.alive(wrong_token)
            with pytest.raises(Conflict, match="Cannot verify ownership"):
                await runtime.stop(wrong_token)
            assert process_identity(child_pid)
        finally:
            await runtime.stop(record)
    asyncio.run(scenario())
