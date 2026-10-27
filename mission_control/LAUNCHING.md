# Mission Control launch workflows

For UI-driven testing and demos, start the world in one terminal and the
dashboard in another. `ui:=true` leaves robot deployment to the dashboard;
`mode:=deploy` adds click-to-spawn controls without taking ownership of Gazebo.
The dashboard launch also starts Lichtblick and its read-only ROS bridge by default.
The normal simulation launch and the external dashboard mode remain available
for manual debugging.

## Install dependencies and build

Inside an existing devcontainer, run this from the repository root:

```bash
bash .devcontainer/scripts/setup_dependencies.sh
```

The devcontainer post-create hook runs this same script automatically. It
installs apt/rosdep dependencies, the Python backend in `.venv/mission-control`,
the pinned pnpm package manager, and the frontend dependencies and built assets.
It reuses compatible Node 22.20+ or 24.12+ when available; otherwise it downloads
and SHA256-checks an official Node 22 runtime into `.venv/frontend-tools`.
Existing containers do not need rebuilding to obtain Node, and no separate
pip, Corepack or pnpm installation command is required.

**Rebuild the ROS packages after pulling these launch changes**, using your
normal `cb`/`colcon build`, then source the resulting workspace in each terminal.
For an isolated build instead:

```bash
source /opt/ros/humble/setup.bash
cd rs1_ws
colcon build --build-base build-mission-control \
  --install-base install-mission-control \
  --packages-up-to rs1_dashboard rs1_mission --parallel-workers 3
cd ..
```

The ROS build does **not** require the web backend's pip dependencies; installing
web dependencies and building ROS can happen in either order. ROS build
requirements come from apt/rosdep. Colcon installs ROS packages and launch files;
pip installs the web service; pnpm builds the React/TypeScript frontend. Keep
ordinary colcon builds outside the dashboard virtual environment.

Source the matching install in every launch terminal, from the repository root:

```bash
source /opt/ros/humble/setup.bash
source rs1_ws/install/setup.bash
```

For the isolated build above, use `rs1_ws/install-mission-control/setup.bash`
instead. Zsh users can use the matching `setup.zsh` files. Use the same
`ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` and `RMW_IMPLEMENTATION` in all terminals.
Inherited settings are preserved; the ROS dashboard launch uses domain `0` if
unset, matching normal ROS launches.

If `cb` reports all packages finished and only `byte-compiling is disabled,
skipping`, the build succeeded. Older devcontainers may retain
`PYTHONDONTWRITEBYTECODE=1`; `unset PYTHONDONTWRITEBYTECODE` before `cb` enables
optional `.pyc` caching. Recreate the container to apply the updated configuration
to all terminals. The `41068_ignition_bringup` naming warning is also pre-existing.
Neither warning is a missing dependency error.

## Recommended demo: two terminals

Terminal 1 — Gazebo world, simulation clock and world identity/status:

```bash
ros2 launch rs1_sim rs1_sim.launch.py ui:=true
```

This skips robot spawning and robot-dependent nodes, including alignment. Add
`headless:=True` to run without the Gazebo window. The world and its existing
ground station are loaded by this terminal.

Optionally open both existing RViz displays before deploying:

```bash
ros2 launch rs1_sim rs1_sim.launch.py ui:=true rviz:=true
```

RViz initially has no robot TF, maps or sensor data; missing-frame/topic warnings
are expected until the dashboard spawns the robots and their nodes become ready.
The displays remain with the simulation terminal, and deployment does not open
another pair. `ui:=true` alone leaves RViz off; normal simulation still defaults
to RViz on.

Terminal 2 — dashboard with deployment controls:

```bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy
```

Open **http://localhost:8000**. Do not separately launch `rs1_mission` for this
workflow. Lichtblick and its read-only bridge start with this dashboard command;
the **Telemetry** section is available without an extra terminal. In the dashboard:

1. Choose the registered image of the running world and click **Robot start**.
   Both robots share the selected X/Y; terrain determines their spawn heights.
2. Select **Spawn robots**. This launches the robot models, bridges,
   localisation, navigation, alignment, coordinator and mission executors.
3. Wait for readiness, then select an explicit **Point ROI**. Spawning does not
   start a mission or silently use the coordinator's initial placeholder target.
4. Select **Start Mission**. Use **Cancel Mission** to request cancellation.
5. Once the mission is idle or finished and executor cleanup is complete,
   **Stop robots** stops the deployed robot/mission nodes and removes their
   Gazebo models. The world and ground station remain running. Before deploying
   again, stop and restart terminal 1 with
   `ros2 launch rs1_sim rs1_sim.launch.py ui:=true`. The dashboard can stay open.

**Current Fortress limitation:** recreating the robots' sensor stack a second
time in one Gazebo world instance can crash Ogre2. The dashboard therefore blocks
another deployment into that same instance after **Stop robots**. Restarting the
world clears that restriction. Browser refreshes and dashboard backend restarts
while a deployment is active do not recreate robots and remain non-invasive.

Closing or refreshing the browser, or stopping/restarting the dashboard launch,
leaves the deployed ROS processes running. Reopening the dashboard discovers its
existing deployment rather than spawning duplicates. **Clear trails** clears only
recorded trails and dashboard events. The running mission and alignment remain
unchanged. A restarted backend begins recording new trails.

In deploy mode, the dashboard terminal also streams the deployed ROS logs by
default (`show_logs:=true`). Mission coordinator, both mission executors,
exploration and terrain-following use `log_level:=info`. Supporting nodes such
as Nav2, SLAM and EKF use `stack_log_level:=warn` to reduce routine output.
The simulation terminal continues showing its own world/session logs. These
are terminal logs, not a full log panel in the browser.

For more supporting-node detail, launch the dashboard with
`ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy stack_log_level:=info`.
Use `log_level:=debug` for mission diagnostics, or `stack_log_level:=debug` for
much noisier supporting-node diagnostics. Severity changes apply when spawning
a new deployment; reconnecting does not reconfigure already running nodes.
`show_logs:=false` disables terminal streaming without disabling file logging.
All deployment output remains in `.mission-control/deploy/sessions/<session_id>/ros.log`
(under the configured data directory if overridden). After a dashboard restart,
streaming resumes with new messages rather than replaying the existing history.

The deployment follows the world instance in terminal 1. If that world is
closed or its status disappears, a ROS-side guard stops the deployed stack after
10 seconds without a matching fresh world status, even when the dashboard is
closed. A new world is a new deployment; stale state is not silently adopted.

Only `v2_forest` is currently registered for the dashboard. Selecting or uploading
a calibrated image describes that existing world; it does not load a different
Gazebo world. The ground station stays at the world's configured position. Robot
start, clearance and image selection are locked after spawning until **Stop
robots**. A fresh world launch is required before applying another deployment.
Use a new world launch if you need different world geometry.

Deployment ownership is recorded in
`/tmp/rs1-deploy-<uid>-<domain>/deployment.json`. This per-user, per-domain journal
allows a restarted dashboard to reconnect and prevents a second deployment even
when its `data_dir` changes. Do not delete it to bypass an active deployment.
Plans and ROS output remain under the backend's `sessions/<id>/` directory,
including `ros.log`; the default deploy data directory is
`<repo>/.mission-control/deploy`.

## Telemetry included with the dashboard

The normal `dashboard.launch.py` command starts the pinned, unmodified Lichtblick
web viewer on loopback port **8080** and its read-only Foxglove WebSocket bridge
on **8765**. The normal dependency setup installs both. No additional launch
flag or terminal is needed. Telemetry starts before **Spawn robots**;
robot topics become available when their publishers start. Select **Reset to
starter layout** in the dashboard for registered maps/TF, both cameras, downward range,
commanded vertical velocity and mission status. Stopping the dashboard launch
also stops its viewer and bridge. Browser refreshes leave all services running.

Forward ports **8080** and **8765** in addition to dashboard port **8000** for a
remote devcontainer. See [telemetry documentation](telemetry/README.md) for
browser-reachable URL overrides, HTTPS/WSS proxies, layout persistence, and
using an existing desktop viewer.

To keep telemetry running across dashboard restarts, disable the integrated
services and run the separate launch in another sourced terminal:

```bash
# Dashboard terminal
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy telemetry:=false

# Separate telemetry terminal, in the same ROS domain
ros2 launch rs1_dashboard telemetry.launch.py
```

The separate launch owns only telemetry; stopping it leaves the world, robots,
dashboard and mission running. Also use `telemetry:=false` when telemetry
services already run elsewhere. Run only one viewer/bridge pair per ROS domain.

## Manual debugging: normal simulation plus external dashboard

Terminal 1 — complete simulation, robots, localisation, navigation and alignment:

```bash
ros2 launch rs1_sim rs1_sim.launch.py
```

This retains the normal launch behaviour. Spawn coordinates remain launch
arguments such as `husky_x`, `husky_y`, `husky_z`, `parrot_x`, `parrot_y` and
`parrot_z`. Add `headless:=True rviz:=False` to suppress Gazebo and RViz windows.
You can run and debug this simulation without the dashboard.

Terminal 2 — coordinator and both mission executors, when needed:

```bash
ros2 launch rs1_mission rs1_mission.launch.py
```

Terminal 3 — optional dashboard attaching to those nodes:

```bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=external
```

`external` remains the dashboard launch's default. It starts the web service,
its ROS client, Lichtblick and the read-only telemetry bridge, displays estimated
poses, and sends ROI/start/cancel requests.
It does not start or stop the separately launched ROS processes. The coordinator
retains its `roi_x`, `roi_y`, `roi_z` and `roi_frame` launch arguments. An untouched
ROI preserves that configured frame and Z; editing a world ROI uses terrain Z.

Robot and ground-station placement stay with the simulation launch in this mode.
After completion, cancellation or failure, restart the mission launch in terminal
2 to run another mission. When restarting the simulator, also restart the mission
launch so its TF buffers and goals belong to the new world. Closing/restarting
the dashboard and **Clear trails** leave both terminals running.

Both normal simulation and mission launches default to `session_id:=manual`.
If overriding it, pass the same identifier to both launches.

## Alignment ownership

The normal `rs1_sim.launch.py` starts one alignment node with the robot stack.
`ui:=true` skips that stack; deploy mode starts alignment later, together with the
robots. Restarting the dashboard never repeats registration. Do not also run
`ros2 run rs1_alignment alignment` alongside either workflow.

RViz uses global `/tf` and `/tf_static` so it can transform the world-frame ROI
marker into either robot's map frame. Its Fixed Frame can remain `husky1_map`
or `parrot1_map`. If an older RViz launch says `frame [world] does not exist`
after alignment is ready, close those displays and launch
`ros2 launch rs1_sim rviz.launch.py` from the rebuilt, sourced workspace.
The world and mission can keep running. Before robot deployment and alignment,
the missing-world message is expected.

For a custom normal launch that supplies its own alignment component, opt out:

```bash
ros2 launch rs1_sim rs1_sim.launch.py alignment:=False
```

## Dashboard modes and options

| Mode | Simulation responsibility | Dashboard restart |
|---|---|---|
| `deploy` | Deploys robots and mission nodes into an existing `ui:=true` world. | Reconnects to the persistent deployment; world and robot nodes continue. |
| `external` (default) | Connects to simulation and mission nodes launched manually. | Reconnects; externally launched processes continue. |
| `managed` | Starts the world, robots and mission together from a complete saved plan. | Backend shutdown stops its owned simulation. Browser refresh is non-invasive. |

| Argument | Default | Purpose |
|---|---|---|
| `mode` | `external` | Select `external`, `deploy` or `managed`. |
| `host` | `127.0.0.1` | HTTP bind address. |
| `port` | `8000` | HTTP port. |
| `telemetry` | `true` | Start Lichtblick and its read-only bridge with the dashboard. Set `false` for separate or existing telemetry services. |
| `telemetry_host` | `127.0.0.1` | Bind address for the integrated viewer and bridge. |
| `viewer_port` | `8080` | Lichtblick HTTP port. |
| `bridge_port` | `8765` | Foxglove WebSocket bridge port. |
| `repo_dir` | Auto-detected, or `MISSION_REPO` | Source repository containing terrain assets and frontend build. |
| `python_executable` | Repository `.venv/mission-control/bin/python`, otherwise launch Python | Interpreter with backend and ROS modules installed. |
| `data_dir` | Mode-specific directory under `<repo>/.mission-control` | `external/` for external, `deploy/` for deploy, root for managed; honors `MISSION_DATA`. |

For a virtual environment elsewhere or a copied install outside the source
workspace, specify paths explicitly:

```bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy \
  repo_dir:=/robotics-studio-1 \
  python_executable:=/path/to/venv/bin/python \
  port:=8000
```

Keep the default localhost binding unless your development environment requires
otherwise; the web service has no authentication. Use one backend per ROS domain
and no Uvicorn reload or multiple-worker options.

## Port already in use

If Uvicorn exits with `[Errno 98] ... ('127.0.0.1', 8000): address already in use`,
another process has reserved that port. The repository's Linux devcontainer uses
`network_mode: host`, so its services already use the host's localhost directly.
VS Code port forwarding is unnecessary and can itself occupy the dashboard port.
The devcontainer configuration no longer requests forwarding for ports 8000/5173
and marks both ports to be ignored by automatic forwarding.

For a VS Code window opened with the older configuration, open the **Ports**
panel, right-click port **8000**, and choose **Stop Forwarding**. Remove **5173**
there too if present. Retry the dashboard launch; no ROS rebuild is required.
On a subsequent devcontainer reopen, VS Code will read the updated configuration.
Use VS Code's controls to close forwards rather than force-killing its process.

If the Ports panel does not list 8000 but a host-side
`ss -ltnp '( sport = :8000 )'` identifies `code` as the listener, another VS Code
window or an old shared-process tunnel may own it. Check the other windows first.
If none lists it, save your work and exit VS Code completely using **File → Exit**,
then reopen the devcontainer. A container rebuild or window reload can leave the
shared host process alive. Exiting can stop this devcontainer and its terminals,
so restart your ROS launches afterward. The alternate port below avoids needing
to close the editor immediately.

If another application needs port 8000, run the dashboard on a free port instead:

```bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy port:=8001
```

Then open `http://localhost:8001`. The ROS domain and simulation launch stay the
same. Only one dashboard backend may own a given ROS domain.

## Optional managed workflow

In an otherwise unused ROS domain, without a separate simulation or mission
launch:

```bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=managed
```

The dashboard validates robot start, ROI and station placement, writes a session
world, starts the combined ROS launch, waits for readiness and starts the mission.
**New session** stops that owned simulation. Unlike deploy mode, shutting down
the backend also stops its owned simulation. Browser refreshes leave it running.

Direct `python -m uvicorn mission_control.app:app` retains the managed default.
Set `MISSION_MODE=deploy` or `MISSION_MODE=external` explicitly to use another
workflow, and set ROS environment variables to match the running world/stack.
Direct Uvicorn otherwise defaults to domain `42` and localhost-only discovery.
Direct Uvicorn starts only the backend; use the separate telemetry launch if
you want Lichtblick in this advanced workflow.

## Optional frontend development

After setup, expose the private frontend tools in a Bash or Zsh terminal:

```bash
source .venv/frontend-tools/env.sh
cd mission_control/frontend
pnpm dev
```

Use port 5173 for Vite development while the dashboard backend runs on port 8000.
`pnpm test` runs frontend tests; `pnpm build` updates the production assets served
by the dashboard. Dependency installation is identical for all three modes.
