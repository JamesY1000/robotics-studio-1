"""Show deployment output without tying the ROS process lifetime to the dashboard."""
import asyncio
from pathlib import Path


class DeploymentLogTail:
    def __init__(self, deployment):
        self.path = (deployment or {}).get("log_path")
        self.offset = 0
        # On dashboard restart, show new messages only. The complete history
        # remains in ros.log; replaying it could flood the operator's terminal.
        if self.path:
            try:
                self.offset = Path(self.path).stat().st_size
            except FileNotFoundError:
                pass
        self.announced = False

    def read(self, deployment):
        path = (deployment or {}).get("log_path")
        if path != self.path:
            # A new deployment needs its startup messages as well.
            self.path, self.offset, self.announced = path, 0, False
        if not path:
            return ""
        try:
            with open(path, "rb") as log:
                if Path(path).stat().st_size < self.offset:
                    self.offset = 0
                log.seek(self.offset)
                # Bound each read so a burst of logs does not monopolise polling.
                output = log.read(65536)
                self.offset = log.tell()
        except FileNotFoundError:
            return ""  # The launch may not have created its log yet.
        banner = "" if self.announced else f"\n[deployment logs] Following {path}\n"
        self.announced = True
        return banner + output.decode("utf-8", errors="replace")


async def stream_deployment_logs(manager, tail):
    def emit(deployment):
        output = tail.read(deployment)
        if output:
            print(output, end="", flush=True)

    last_error = None
    while True:
        try:
            # File and terminal I/O run away from the mission API's event loop.
            # ROS still writes directly to its own file if this viewer stops.
            await asyncio.to_thread(emit, manager.deployment)
            last_error = None
        except OSError as error:
            if str(error) != last_error:
                print(f"[deployment logs] Cannot stream log: {error}", flush=True)
                last_error = str(error)
        await asyncio.sleep(0.5)
