#!/usr/bin/env bash

# Install only the Python dashboard backend. ROS builds and the frontend build
# remain separate operations; this script does not start any application.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
dashboard_venv="${repo_root}/.venv/mission-control"

# Humble's binary rclpy modules require the system Python. Expose ROS packages
# without changing the interpreter or dependencies used by ordinary colcon builds.
/usr/bin/python3 -m venv --system-site-packages "${dashboard_venv}"
"${dashboard_venv}/bin/python" -m pip install --upgrade pip 'setuptools>=65,<80' wheel
# Editable install also reads backend/requirements.txt through pyproject.toml.
# The test extra supplies pytest/httpx for the backend's focused test suite.
"${dashboard_venv}/bin/python" -m pip install --no-build-isolation \
    -e "${repo_root}/mission_control/backend[test]"

printf '%s\n' 'Dashboard backend installed in .venv/mission-control.'
