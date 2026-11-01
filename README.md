# Robotics Studio 1

Repository for the UTS subject Robotics Studio 1 group 2.

Members are:
- James Yuan
- Timothy Ollerton
- Nikhil Kumar
- Noah Nourdine
- Thejas Madathil

This repository is a common place to store and maintain our code, documentation, and setup instructions for the project throughout the semester.

## Mission Control

For UI-driven demos, launch the world and dashboard in two sourced devcontainer
terminals:

```bash
# To offload OpenGL rendering to an Nvidia graphics card, run in whichever terminal you need it for (likely the rs1_sim.launch.py and the rviz one, but you can do for all of them tbh)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Terminal 1: world only
ros2 launch rs1_sim rs1_sim.launch.py ui:=true

# Terminal 2: click-to-spawn, ROI selection and mission controls
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy

# One shared RViz window (lighter mission overview)
ros2 launch rs1_sim rviz.launch.py combined:=true

# Or keep the original separate robot windows
ros2 launch rs1_sim rviz.launch.py
```

The combined view uses the shared `world` frame, groups displays under Husky,
Parrot and Mission, and renders at 15 FPS. Camera/depth and 3D LiDAR displays
start disabled; enable them in the Displays panel when needed. Only Husky's
SLAM map starts visible to avoid overlapping maps; Parrot's map is available in
its Maps group. Use the dashboard for mission commands or the separate windows
for robot-specific Nav2 controls. The robot stacks must be running with their
normal world alignment for the shared view to show data.

The same option is available on the simulation and robot bringup launches:
`ros2 launch rs1_sim rs1_sim.launch.py combined:=true`. For dashboard deployment
with RViz opened alongside the world, add `ui:=true rviz:=true combined:=true`.

Open **http://localhost:8000**, choose robot positions, select **Spawn robots**,
then choose an ROI and **Start Mission**. The dashboard launches the robot stacks
and mission nodes; no separate mission launch is needed. Restarting the dashboard
leaves that deployment running. **Stop robots** explicitly removes the deployment
while preserving the world. Restart the world terminal with `ui:=true` before
deploying again: Gazebo Fortress currently cannot reliably recreate this sensor
stack twice in one world instance.

For manual debugging, `rs1_sim.launch.py` without `ui:=true` still starts the
complete simulation and robots. Launch mission nodes separately when needed and
optionally attach the dashboard with `mode:=external`. An optional `managed`
mode retains the dashboard-owned, all-in-one simulation flow. Rebuild with `cb`
and source the workspace after updating launch files. See [launch commands](mission_control/LAUNCHING.md) and
[installation, operation and validation](mission_control/README.md).

## Project Workflow (Quick Start)

1. Pull the latest changes from main.
2. Create a branch for your task.
3. Develop and test locally in the devcontainer.
4. Open a pull request with a clear description.
5. Get at least one teammate review.
6. Merge only when checks pass.

## Development Environment

I've already set up a docker devcontainer that will be able to handle everything this subject throws at us plus whatever we want to run/setup, but to be honest you can continue using whatever setup you've currently got if that's easier/more comfortable for you.

### Docker setup (if you want to use the devcontainer)

- **Docker installed and running**

  1. Open the [Docker Engine installation guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

  2. Under **Install from the apt repository**, complete the following steps:

     1. Set up Docker's apt repository.
     2. Install the Docker packages.
     3. Verify that the installation was successful by running the `hello-world` image.

  3. Continue to [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/) and complete the following steps:

     1. Create the `docker` group.
     2. Add your user to the `docker` group.
     3. Log out and log back in so that your group membership is re-evaluated.
        - If you are using an Azure VM, you may need to restart the VM completely.
     4. Verify that you can run Docker commands without `sudo`.

- VS Code with the 'Dev Containers' extension installed.
- A local workspace directory at ~/git.
- A valid git config on your machine.
- ccache installed/configured (recommended).

If your computer has an NVIDIA GPU:
- Install NVIDIA Container Toolkit (Docker NVIDIA runtime).

If your computer does not have an NVIDIA GPU, or NVIDIA runtime setup fails:
- Open the .env file.
- Change container_runtime from nvidia to runc.

To allow X11 forwarding to the container which you'll need for gui applications like rviz, run the command on host before rebuilding the devcontainer (in VSCode). This is a temporary solution till I figure out how to allow forwarding more cleanly:
```bash
xhost +SI:localuser:$(whoami)
```

If this doesn't work, run this instead on host:
```bash
xhost +local:
```

### Build the Devcontainer Images

From the .devcontainer directory:

1. Build the base image:

```bash
docker compose --profile devcontainer-build build rs1-devcontainer-base
```

2. Build the user image:

```bash
docker compose build rs1-devcontainer-user
```

3. Open the repo in VS Code and select Reopen in Container.

After initial setup, you usually only need to open the repository in VS Code and reopen in container. If you prefer to run multiple terminal windows, you can use lazydocker and enter the container this way.

The devcontainer post-create hook installs the project dependencies and builds
the dashboard frontend. In an existing container, or one opened without that
hook, run the same setup from the repository root:

```bash
bash .devcontainer/scripts/setup_dependencies.sh
```

This handles apt/rosdep dependencies, the Python backend in
`.venv/mission-control`, Node/pnpm and the frontend's locked dependency install
and build. A compatible existing Node is reused; otherwise the script installs
a SHA256-checked official Node runtime privately under `.venv/frontend-tools`.
No separate pip/pnpm commands or devcontainer rebuild are required for this
setup. Build the ROS source packages separately with your normal `cb`/`colcon
build`, then source the matching workspace install before launching. ROS can be
built before or after installing the dashboard's web dependencies.

Dashboard Python requirements are in `mission_control/backend/requirements.txt`;
frontend dependencies are in `mission_control/frontend/package.json` and its
pnpm lockfile. The base image's `.devcontainer/requirements.txt` contains global
Python development tools. See the Mission Control installation guide linked
above for the two-terminal demo, manual debugging and frontend development.

## Branching and Commit Conventions

Use one branch per issue or feature.

Suggested branch prefixes:
- feature/<short-description>
- fix/<short-description>
- test/<short-description>
- docs/<short-description>

Commit message style:
- Keep messages concise and descriptive.
- Start with an action verb.
- Mention subsystem when relevant.

Example:
- Add retry logic for UWB packet timeout

## Pull Request Guidelines

Before opening a PR:
- Rebase or merge latest main into your branch.
- Confirm your branch builds successfully.
- Update docs if behavior/setup changed.

PR expectations:
- Clearly defined and focused scope.
- Clear summary of what changed and why.
- At least one teammate review before merge.

## Team Rules

1. Never push directly to main.
2. Pull the latest main before starting work.
3. Use one branch per issue or feature.
4. Use descriptive branch and commit names.
5. Keep pull requests focused and reasonably small.
6. At least one teammate reviews each pull request.
7. Do not merge code that does not build.
8. Merge frequently instead of integrating everything near the deadline.
9. Document any dependency assumptions and setup procedures.
10. Never commit secrets, generated build folders, or large datasets.

## Definition of Done

A task is done when:
- Code is implemented (working) and reviewed.
- Code does not break any other part of overall system.
- Relevant docs are updated.
