"""Short startup gate, replaced by ros2 launch after ownership is durable."""
import json
import os
from pathlib import Path
import sys
import time

from .deployment_runtime import process_identity


def main():
    path, session = Path(sys.argv[1]), sys.argv[2]
    for _ in range(100):
        try:
            record = json.loads(path.read_text())
            identity = process_identity(os.getpid())
            if (record.get("session_id") == session and record.get("pid") == os.getpid()
                    and identity and all(record.get(key) == value for key, value in identity.items())
                    and record.get("phase") == "spawning"):
                os.execvpe(record["command"][0], record["command"], os.environ)
        except (OSError, ValueError):
            pass
        time.sleep(0.1)
    raise SystemExit("Deployment was not recorded; no robots were launched")


if __name__ == "__main__":
    main()
