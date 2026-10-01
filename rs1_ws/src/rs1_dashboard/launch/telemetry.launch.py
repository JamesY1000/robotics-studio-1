"""Read-only telemetry: upstream Lichtblick web viewer and a ROS WebSocket bridge."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def start_telemetry(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    configured = value("repo_dir")
    roots = [Path(configured).expanduser()] if configured else [
        candidate for base in (Path.cwd(), Path(__file__).resolve().parent)
        for candidate in (base, *base.parents)]
    root = next((path for path in roots if (path / "mission_control/frontend/package.json").is_file()), None)
    if root is None:
        raise RuntimeError("Cannot find the RS1 repository; set repo_dir:=/path/to/robotics-studio-1")
    ports = {name: int(value(name)) for name in ("viewer_port", "bridge_port")}
    if any(not 1 <= port <= 65535 for port in ports.values()):
        raise ValueError("Telemetry ports must be between 1 and 65535")
    if ports["viewer_port"] == ports["bridge_port"]:
        raise ValueError("Viewer and bridge need different ports")
    actions = []
    if value("viewer").lower() in ("true", "1"):
        directory = root / ".venv/lichtblick/1.29.1"
        if not (directory / "index.html").is_file():
            raise RuntimeError("Lichtblick assets are missing. Run bash .devcontainer/scripts/setup_telemetry_viewer.sh")
        executable = Path(get_package_prefix("rs1_dashboard")) / "lib/rs1_dashboard/telemetry_viewer.py"
        actions += [
            LogInfo(msg=f"Lichtblick viewer: http://{value('viewer_host')}:{ports['viewer_port']}"),
            ExecuteProcess(cmd=[str(executable), "--directory", str(directory),
                                "--host", value("viewer_host"), "--port", str(ports["viewer_port"])],
                           output="screen"),
        ]
    if value("bridge").lower() in ("true", "1"):
        actions += [
            LogInfo(msg=f"Telemetry bridge: ws://{value('bridge_host')}:{ports['bridge_port']}"),
            Node(package="foxglove_bridge", executable="foxglove_bridge",
                 namespace="rs1_telemetry", name="foxglove_bridge", output="screen",
                 parameters=[{
                     "address": value("bridge_host"), "port": ports["bridge_port"],
                     # Observation only: mission commands remain in the dashboard.
                     "capabilities": ["connectionGraph", "assets"],
                     "remote_access": False,
                     "topic_whitelist": ["^/(husky1|parrot1)/.*$", "^/(tf|tf_static|clock)$",
                                         "^/mission/.*$", "^/simulation/session$"],
                     "use_sim_time": False,
                 }]),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("repo_dir", default_value=os.environ.get("MISSION_REPO", "")),
        DeclareLaunchArgument("viewer", default_value="true", description="Serve the pinned upstream Lichtblick web app."),
        DeclareLaunchArgument("bridge", default_value="true", description="Start the read-only ROS telemetry bridge."),
        DeclareLaunchArgument("viewer_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("viewer_port", default_value="8080"),
        DeclareLaunchArgument("bridge_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("bridge_port", default_value="8765"),
        OpaqueFunction(function=start_telemetry),
    ])
