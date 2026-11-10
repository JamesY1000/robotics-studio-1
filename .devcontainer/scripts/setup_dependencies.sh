#!/usr/bin/env bash

# One entry point for the existing ROS Humble devcontainer: ROS dependencies,
# perception runtime, Python backend, Node/pnpm, dashboard assets, and telemetry.
# This does not build the ROS source packages or start any application.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
tools_dir="${repo_root}/.venv/frontend-tools"
node_version="22.23.3"
download_dir=""
trap 'if [[ -n "${download_dir}" ]]; then rm -rf -- "${download_dir}"; fi' EXIT

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    printf '%s\n' "Run this script inside the RS1 devcontainer with ROS ${ros_distro} installed." >&2
    exit 1
fi

# ROS environment scripts read optional variables before defining them.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ros_distro}/setup.bash"
set -u

printf '%s\n' '[1/6] Initialising git submodules'
# explore_lite (rs1_ws/src/extern/m-explore-ros2) and Forest3D are source
# dependencies. Fetch them before rosdep scans rs1_ws/src so their package.xml
# files and dependencies are visible to the build.
git -C "${repo_root}" submodule update --init --recursive

printf '%s\n' '[2/6] Installing system and ROS dependencies'
if [[ "${EUID}" -eq 0 ]]; then
    elevate=()
else
    elevate=(sudo -H)
fi
"${elevate[@]}" apt-get update
"${elevate[@]}" apt-get install -y --no-install-recommends \
    ca-certificates curl xz-utils python3-pip python3-venv
rosdep fix-permissions || true
rosdep update --rosdistro "${ros_distro}"
rosdep install --from-paths "${repo_root}/rs1_ws/src" --ignore-src -r -y \
    --rosdistro "${ros_distro}"

# The ROS perception executables use system Python, not the dashboard venv.
printf '%s\n' 'Installing the ONNX perception runtime'
"${elevate[@]}" /usr/bin/python3 -m pip install \
    -r "${repo_root}/rs1_ws/src/rs1_perception/requirements/onnx.txt"

printf '%s\n' '[3/6] Installing the Python dashboard backend'
bash "${repo_root}/.devcontainer/scripts/setup_dashboard_backend.sh"

printf '%s\n' '[4/6] Preparing Node.js and the pinned pnpm package manager'
mkdir -p "${tools_dir}"
node_usable() {
    command -v node >/dev/null 2>&1 && command -v npm >/dev/null 2>&1 &&
        node -e 'const [major, minor] = process.versions.node.split(".").map(Number);
            process.exit((major === 22 && minor >= 20) || (major === 24 && minor >= 12) ? 0 : 1)'
}

# A rebuilt devcontainer supplies Node via its feature. Older containers get a
# repository-local runtime from the official Node distribution, without nvm,
# shell profile changes, or a global npm installation.
if ! node_usable; then
    case "$(uname -m)" in
        x86_64) node_arch=x64 ;;
        aarch64|arm64) node_arch=arm64 ;;
        *) printf '%s\n' 'Unsupported architecture; install Node 22.20+ or 24.12+ and npm first.' >&2; exit 1 ;;
    esac
    node_root="${tools_dir}/node-v${node_version}-linux-${node_arch}"
    if [[ ! -x "${node_root}/bin/node" ]]; then
        download_dir="$(mktemp -d)"
        archive="node-v${node_version}-linux-${node_arch}.tar.xz"
        base_url="https://nodejs.org/dist/v${node_version}"
        curl --fail --show-error --location --retry 3 \
            "${base_url}/SHASUMS256.txt" -o "${download_dir}/SHASUMS256.txt"
        curl --fail --show-error --location --retry 3 \
            "${base_url}/${archive}" -o "${download_dir}/${archive}"
        awk -v archive="${archive}" '$2 == archive {print; found=1} END {if (!found) exit 1}' \
            "${download_dir}/SHASUMS256.txt" > "${download_dir}/selected.sha256"
        (cd "${download_dir}" && sha256sum --check selected.sha256)
        tar -xJf "${download_dir}/${archive}" -C "${download_dir}"
        mv -- "${download_dir}/node-v${node_version}-linux-${node_arch}" "${node_root}"
    fi
    export PATH="${node_root}/bin:${PATH}"
fi
node_usable
node_bin="$(dirname -- "$(command -v node)")"
pnpm_spec="$(node -p "require(process.argv[1]).packageManager" "${repo_root}/mission_control/frontend/package.json")"
if [[ ! "${pnpm_spec}" =~ ^pnpm@[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    printf '%s\n' 'frontend/package.json must pin an exact pnpm packageManager version.' >&2
    exit 1
fi
npm install --prefix "${tools_dir}" --no-save --package-lock=false \
    --ignore-scripts --no-audit --no-fund "${pnpm_spec}"
export PATH="${tools_dir}/node_modules/.bin:${PATH}"
# Optional convenience for frontend development in subsequent Bash/Zsh terminals.
# shellcheck disable=SC2016 # Expand PATH when env.sh is sourced, not now.
printf 'export PATH=%q:"${PATH}"\n' "${tools_dir}/node_modules/.bin:${node_bin}" > "${tools_dir}/env.sh"

printf '%s\n' '[5/6] Installing frontend dependencies and building the dashboard'
cd "${repo_root}/mission_control/frontend"
# Build tools are devDependencies even though the generated site is production.
env -u NODE_ENV CI=true pnpm install --frozen-lockfile --prod=false
env -u NODE_ENV CI=true pnpm build

printf '%s\n' '[6/6] Installing the pinned Lichtblick telemetry viewer'
bash "${repo_root}/.devcontainer/scripts/setup_telemetry_viewer.sh"

printf '%s\n' 'Dependencies installed and dashboard assets built.' \
    'Next: build the ROS workspace with cb/colcon, source its install, then follow mission_control/LAUNCHING.md.' \
    'For optional frontend development: source .venv/frontend-tools/env.sh from the repository root.'
