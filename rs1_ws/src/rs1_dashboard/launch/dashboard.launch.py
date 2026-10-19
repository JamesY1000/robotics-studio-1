"""Launch the dashboard and telemetry; attach to existing mission ROS by default."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def repository_root(configured):
    """Find the source assets with regular or symlink colcon installs."""
    if configured:
        candidates = [Path(configured).expanduser().resolve()]
    else:
        sim_share = Path(get_package_share_directory("rs1_sim"))
        roots = [
            Path(__file__).resolve().parent,
            Path.cwd(),
            sim_share.resolve(),
            (sim_share / "config/map_presets.json").resolve().parent,
        ]
        candidates = [
            candidate for root in roots for candidate in (root, *root.parents)
        ]
    for candidate in candidates:
        if (
            candidate / "mission_control/backend/mission_control/app.py"
        ).is_file() and (
            candidate / "rs1_ws/src/rs1_sim/config/map_presets.json"
        ).is_file():
            return candidate
    raise RuntimeError(
        "Cannot find the RS1 source repository. Set repo_dir:=/path/to/robotics-studio-1."
    )


def start_dashboard(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    root = repository_root(value("repo_dir"))
    mode = value("mode")
    python = value("python_executable")
    if not python:
        venv_python = root / ".venv/mission-control/bin/python"
        python = str(venv_python) if venv_python.is_file() else sys.executable
    default_data = root / ".mission-control"
    if mode == "external":
        default_data /= "external"
    elif mode == "deploy":
        default_data /= "deploy"
    data_dir = value("data_dir") or str(default_data)
    port = int(value("port"))
    if not 1 <= port <= 65535:
        raise ValueError("port must be between 1 and 65535")
    telemetry_enabled = value("telemetry").lower() in ("true", "1")
    if telemetry_enabled and port in (
        int(value("viewer_port")),
        int(value("bridge_port")),
    ):
        raise ValueError("Dashboard, viewer and bridge need different ports")
    environment = {
        "MISSION_MODE": mode,
        "MISSION_REPO": str(root),
        "MISSION_DATA": str(Path(data_dir).expanduser().resolve()),
        # Bare ROS launches use domain 0 and permit network discovery. Prevent
        # direct-Uvicorn convenience defaults from isolating this third terminal.
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY", "0"),
        "MISSION_TELEMETRY_VIEWER_URL": value("viewer_url"),
        "MISSION_TELEMETRY_BRIDGE_URL": value("bridge_url"),
        "MISSION_TELEMETRY_VIEWER_PORT": value("viewer_port"),
        "MISSION_TELEMETRY_BRIDGE_PORT": value("bridge_port"),
    }
    actions = []
    if telemetry_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(get_package_share_directory("rs1_dashboard"))
                        / "launch/telemetry.launch.py"
                    )
                ),
                launch_arguments={
                    "repo_dir": str(root),
                    "viewer_port": value("viewer_port"),
                    "bridge_port": value("bridge_port"),
                    "viewer_host": value("telemetry_host"),
                    "bridge_host": value("telemetry_host"),
                }.items(),
            )
        )
    return actions + [
        LogInfo(msg=f"Dashboard ({mode}): http://{value('host')}:{port}"),
        ExecuteProcess(
            cmd=[
                python,
                "-m",
                "uvicorn",
                "mission_control.app:app",
                "--host",
                value("host"),
                "--port",
                str(port),
                "--no-access-log",
            ],
            cwd=str(root),
            additional_env=environment,
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="external",
                choices=["external", "deploy", "managed"],
                description="External attaches to ROS; deploy spawns into a ui:=true world; managed launches everything.",
            ),
            DeclareLaunchArgument(
                "host",
                default_value="127.0.0.1",
                description="HTTP bind address; localhost is recommended.",
            ),
            DeclareLaunchArgument(
                "port", default_value="8000", description="HTTP port."
            ),
            DeclareLaunchArgument(
                "telemetry",
                default_value="true",
                description="Launch the Lichtblick viewer and read-only bridge; set false when launched separately.",
            ),
            DeclareLaunchArgument(
                "telemetry_host",
                default_value="127.0.0.1",
                description="Bind address for the telemetry services.",
            ),
            DeclareLaunchArgument(
                "viewer_port",
                default_value=os.environ.get("MISSION_TELEMETRY_VIEWER_PORT", "8080"),
            ),
            DeclareLaunchArgument(
                "bridge_port",
                default_value=os.environ.get("MISSION_TELEMETRY_BRIDGE_PORT", "8765"),
            ),
            DeclareLaunchArgument(
                "viewer_url",
                default_value=os.environ.get("MISSION_TELEMETRY_VIEWER_URL", ""),
                description="Browser-accessible viewer URL; default uses browser hostname and viewer_port.",
            ),
            DeclareLaunchArgument(
                "bridge_url",
                default_value=os.environ.get("MISSION_TELEMETRY_BRIDGE_URL", ""),
                description="Browser-accessible WebSocket URL; default uses browser hostname and bridge_port.",
            ),
            DeclareLaunchArgument(
                "repo_dir",
                default_value=os.environ.get("MISSION_REPO", ""),
                description="RS1 repository root; automatically found when omitted.",
            ),
            DeclareLaunchArgument(
                "python_executable",
                default_value="",
                description="Python with backend installed; defaults to the repository venv if present.",
            ),
            DeclareLaunchArgument(
                "data_dir",
                default_value=os.environ.get("MISSION_DATA", ""),
                description="Session data directory; external and managed defaults are separate.",
            ),
            OpaqueFunction(function=start_dashboard),
        ]
    )
