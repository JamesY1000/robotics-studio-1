"""Detached ROS launches with a durable identity; no background manager service."""
import asyncio
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

from .session import Conflict


def process_stat(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return {"state": fields[0], "group": int(fields[2]), "session": int(fields[3]),
                "process_start": fields[19]}
    except (OSError, IndexError, ValueError):
        return None


def process_identity(pid):
    stat = process_stat(pid)
    if not stat or stat["state"] == "Z":
        return None
    return {"process_start": stat["process_start"],
            "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}


def has_ownership_token(pid, token):
    try:
        marker = f"RS1_DEPLOYMENT_TOKEN={token}".encode()
        return marker in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
    except OSError:
        return False


class DeploymentRuntime:
    def __init__(self, path=None):
        # Shared across data directories, so a backend cannot forget a live deployment
        # simply by changing data_dir. The app holds the existing per-domain lock.
        self.path = Path(path) if path else Path(
            f"/tmp/rs1-deploy-{os.getuid()}-{os.environ.get('ROS_DOMAIN_ID', '0')}/deployment.json")
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.children = []

    def read(self):
        if not self.path.exists():
            return None
        try:
            return json.loads(self.path.read_text())
        except (OSError, ValueError) as exc:
            raise Conflict(f"Cannot read deployment ownership record {self.path}: {exc}") from exc

    def save(self, record):
        temporary = self.path.with_suffix(".tmp")
        with temporary.open("w") as stream:
            json.dump(record, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.path)

    def alive(self, record):
        return self.group_alive(record)

    def group_alive(self, record, strict=False):
        """Prove ownership even when ros2 launch exits before its children.

        A surviving member keeps its process group/session ID reserved. The
        inherited random token also prevents mistaking a later reused group
        for this deployment after a long dashboard outage.
        """
        self.children[:] = [child for child in self.children if child.poll() is None]
        if not record or not record.get("pid"):
            return False
        if record.get("boot_id") != Path("/proc/sys/kernel/random/boot_id").read_text().strip():
            return False
        leader = process_stat(record["pid"])
        if leader:
            if leader["process_start"] != record.get("process_start"):
                return False
            if leader["group"] != record["pid"] or leader["session"] != record["pid"]:
                if strict:
                    raise Conflict("Deployment process group identity changed; refusing to signal it")
                return False
            if leader["state"] != "Z":
                return True

        try:
            started = int(record["process_start"])
        except (KeyError, TypeError, ValueError):
            return False
        members = []
        for path in Path("/proc").iterdir():
            if not path.name.isdecimal():
                continue
            stat = process_stat(path.name)
            if (stat and stat["state"] != "Z" and stat["group"] == record["pid"]
                    and stat["session"] == record["pid"] and int(stat["process_start"]) >= started):
                members.append(int(path.name))
        if not members:
            return False
        token = record.get("ownership_token")
        if token and any(has_ownership_token(pid, token) for pid in members):
            return True
        if strict:
            raise Conflict("Cannot verify ownership of the remaining robot process group; refusing to signal it")
        return False

    def launch(self, record, command, env):
        # Journal before spawning. Child waits for its PID to be journaled before
        # exec'ing ROS, including if the dashboard is killed during Popen.
        record = {**record, "command": command, "pid": None,
                  "ownership_token": secrets.token_hex(24)}
        self.save(record)
        with open(record["log_path"], "ab", buffering=0) as log:
            child = subprocess.Popen(
                [sys.executable, "-m", "mission_control.detached_launch", str(self.path), record["session_id"]],
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
                env={**env, "RS1_DEPLOYMENT_TOKEN": record["ownership_token"]})
        self.children.append(child)
        identity = process_identity(child.pid)
        if identity is None:
            raise RuntimeError("ROS deployment launcher exited before its ownership could be recorded")
        record.update(pid=child.pid, **identity)
        self.save(record)
        return record

    async def stop(self, record):
        # Never signal a PID whose creation identity changed since deployment.
        for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
            if not self.group_alive(record, strict=True):
                return
            try:
                os.killpg(record["pid"], sig)
            except ProcessLookupError:
                return
            deadline = time.monotonic() + timeout
            while self.group_alive(record, strict=True) and time.monotonic() < deadline:
                await asyncio.sleep(0.1)
        if self.group_alive(record, strict=True):
            raise RuntimeError("Robot launch did not stop; inspect its ROS log before retrying")
