"""Viewer connection settings; telemetry processes do not own mission state."""
import os
from urllib.parse import urlsplit


TELEMETRY_NODE = "/rs1_telemetry/foxglove_bridge"


def connection_settings():
    def port(name, default):
        value = int(os.getenv(name, str(default)))
        if not 1 <= value <= 65535:
            raise ValueError(f"{name} must be between 1 and 65535")
        return value

    def url(name, schemes):
        value = os.getenv(name, "").strip()
        if not value:
            return None
        parsed = urlsplit(value)
        if parsed.scheme not in schemes or not parsed.hostname or parsed.username or parsed.password:
            raise ValueError(f"{name} must be a {', '.join(schemes)} URL without credentials")
        return value

    # Resolve default hostnames in the browser, not against the container's host.
    return {
        "viewer_url": url("MISSION_TELEMETRY_VIEWER_URL", ("http", "https")),
        "bridge_url": url("MISSION_TELEMETRY_BRIDGE_URL", ("ws", "wss")),
        "viewer_port": port("MISSION_TELEMETRY_VIEWER_PORT", 8080),
        "bridge_port": port("MISSION_TELEMETRY_BRIDGE_PORT", 8765),
        "layout_url": "/api/telemetry/layout",
    }
