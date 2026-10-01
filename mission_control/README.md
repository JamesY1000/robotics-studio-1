# RS1 Mission Control

A local React/TypeScript dashboard and FastAPI service for the existing ROS 2
Humble / Ignition Gazebo Fortress stack. The first mission phase is traversal to
a point ROI: Parrot explores and hands a ground path to Husky. Sampling and
return-home are not implemented. The UI does not fabricate sensor data, battery
levels, biomass results or completion percentages.

The recommended demo uses **two terminals**: `rs1_sim.launch.py ui:=true` loads
the world, and `dashboard.launch.py mode:=deploy` provides click-to-spawn,
explicit ROI selection, mission controls and the Lichtblick telemetry viewer.
The dashboard launch starts its read-only telemetry bridge automatically.
Deployed robot and mission nodes
survive a dashboard restart. Normal simulation launch and the dashboard's
`external` mode remain available for manual debugging; `managed` retains the
all-in-one backend-owned launch. See [launch commands and ownership](LAUNCHING.md).

## Install and run

Use the repository's devcontainer. Its post-create hook runs the same complete
dependency setup available to an existing container. From the repository root,
**inside the devcontainer**, run:

```bash
bash .devcontainer/scripts/setup_dependencies.sh
```

Skip this command if post-create already completed successfully. The script
installs system prerequisites and ROS dependencies with apt/rosdep, installs the
Python backend in `.venv/mission-control`, installs the pinned pnpm package
manager, and installs and builds the frontend from its lockfile. It reuses a
compatible Node 22.20+ or 24.12+ installation; older containers without one get
a private Node 22 runtime under `.venv/frontend-tools`, downloaded from the
official distribution and checked against its SHA256 checksum. A devcontainer
rebuild is not required just to obtain Node. The script does not start any
application or build the ROS source packages.

**You can build the ROS packages before installing dashboard pip dependencies.**
Colcon builds ROS code and installs the launch files; FastAPI and the other web
packages are needed only when running the dashboard. Build ROS outside the
dashboard virtual environment. The setup script uses `/usr/bin/python3` and
`--system-site-packages` so ROS Humble's Python modules remain available.

Runtime Python dependencies are listed in `backend/requirements.txt`, which
`backend/pyproject.toml` also reads. The existing `.devcontainer/requirements.txt`
continues to install global development tools during the base-image build.
Frontend dependencies live in `frontend/package.json` and `pnpm-lock.yaml`.
They are JavaScript packages installed with pnpm, so a Python `requirements.txt`
cannot install them. The unified script handles both dependency systems.

Rebuild the ROS workspace after these launch changes. You can keep using your
existing `cb` (`colcon build`) command and source its matching install directory. For an
isolated build, run this from the repository root inside the devcontainer:

```bash
source /opt/ros/humble/setup.bash
cd rs1_ws
# Separate directories avoid changing an existing symlink/non-symlink build.
colcon build --build-base build-mission-control \
  --install-base install-mission-control \
  --packages-up-to rs1_dashboard rs1_mission rs1_perception --parallel-workers 3
source install-mission-control/setup.bash
cd ..
```

Then open two terminals at the repository root. In **each** terminal, source
`/opt/ros/humble/setup.bash` and the matching workspace install:
`rs1_ws/install-mission-control/setup.bash` for the isolated build above, or
`rs1_ws/install/setup.bash` for a normal `cb` build. Use the same ROS domain and
middleware settings. Run one command per terminal:

```bash
# Terminal 1: Gazebo world, clock and world status; no robot spawning
ros2 launch rs1_sim rs1_sim.launch.py ui:=true

# Terminal 2: dashboard with robot deployment and mission controls
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy
```

Open **http://localhost:8000** through the forwarded port. This serves the built
frontend and API from the same origin. The same launch serves Lichtblick on
port **8080** and its read-only WebSocket bridge on **8765**; forward these ports
too when the devcontainer is remote. Do not use multiple Uvicorn workers or
`--reload` while using mission controls. Use one backend per ROS domain.
The launch finds `.venv/mission-control/bin/python` automatically; see
[LAUNCHING.md](LAUNCHING.md) for path overrides and other launch arguments.
There is no separate mission launch in deploy mode: **Spawn robots** starts the
robot stacks, alignment, coordinator and executors. Dependencies are identical
for deploy, external and managed modes.

For frontend development, leave the backend on port 8000, then run these commands
from the repository root. The environment script makes the setup's Node and
pinned pnpm available in this terminal; Corepack is not required.

```bash
source .venv/frontend-tools/env.sh
cd mission_control/frontend
pnpm dev
```

Open forwarded port **5173**. Vite proxies `/api` to localhost:8000. Forward both
ports when developing remotely. The existing container uses host networking;
local host access also works. Keep the service bound to localhost: it has no
authentication and is intended for one operator on a trusted development host.

### Advanced: update only the Python backend

The unified setup invokes this helper, which can also be rerun independently:

```bash
bash .devcontainer/scripts/setup_dashboard_backend.sh
```

It installs the local backend package in editable mode, its runtime requirements
and its test dependencies. To update just third-party Python runtime packages
in an already configured environment, use:

```bash
.venv/mission-control/bin/python -m pip install -r mission_control/backend/requirements.txt
```

Installing only this requirements file does not install the local
`mission_control` module. ROS modules come from rosdep/apt and the sourced colcon
workspace.

No ROS installation is required to inspect the UI and validate plans. On a
non-ROS host the backend explicitly reports that its ROS adapter is unavailable
and disables mission launch. The unified setup requires the devcontainer;
for a non-ROS preview, install `mission_control/backend` in an ordinary Python
3.10+ virtual environment and build the frontend with a compatible Node and
the pinned pnpm from `frontend/package.json`. Then run
`.venv/mission-control/bin/python -m uvicorn mission_control.app:app --host 127.0.0.1 --port 8000`
directly instead of the ROS launch.

### Harmless colcon warnings

A summary such as `14 packages finished` with one package reporting stderr is
a successful build. Older devcontainers may retain `PYTHONDONTWRITEBYTECODE=1`,
which causes setuptools to print `byte-compiling is disabled, skipping` for
`rs1_alignment`. Run `unset PYTHONDONTWRITEBYTECODE` before `cb` to allow optional
`.pyc` caching and remove the warning in the current terminal. Recreate the
devcontainer to apply the updated configuration to all terminals. The warning
about `41068_ignition_bringup` starting with a digit is a
pre-existing package naming warning. Neither warning indicates missing dashboard
dependencies.

## Recommended deploy mode: spawn, select ROI, start

1. Start the two terminals above. The dashboard verifies a fresh `ui:=true`
   world before allowing deployment. Choose its registered terrain image.
2. Click **Robot start** on a clearing, or enter coordinates. Both robots share
   X/Y. Review the terrain-derived spawn heights and Parrot sensor clearance.
3. Select **Spawn robots**. Robot models, bridges, localisation, navigation,
   alignment and mission nodes start inside the already-running Gazebo world.
   Robot placement and image selection are now locked.
4. Once the robots are ready, select an explicit **Point ROI** and then
   **Start Mission**. A placeholder coordinator target never starts a mission
   automatically. Watch estimated robot poses and readiness in **Robots & status**.
5. **Cancel Mission** requests cancellation; it is not an emergency stop.
6. After cancellation/completion and executor cleanup, or while the coordinator
   is idle, **Stop robots** stops the deployed ROS nodes and removes their Gazebo
   models. The world and ground station stay running. Before deploying again for
   a new start or mission, restart terminal 1 with
   `ros2 launch rs1_sim rs1_sim.launch.py ui:=true`. The dashboard can stay open.

**Current Fortress limitation:** recreating the robots' sensor stack twice in
one Gazebo world instance can crash Ogre2. After **Stop robots**, another
deployment is blocked until that world is restarted. This does not affect
browser refresh or backend reconnection to a deployment that is already running.

Closing/refreshing the browser or stopping/restarting the dashboard backend is
non-invasive in deploy mode. The ROS launch runs independently and the backend
reconnects to it. **Clear trails** affects only dashboard trails/events. A
restarted backend begins recording new trails; it does not repeat alignment.
A world guard runs with the robot stack and shuts that stack down after 10 seconds
without a matching world heartbeat, including when the dashboard is closed.
A changed world identity invalidates the old deployment.

Terminal 1 selects the actual Gazebo world. Deploy mode currently requires the
standard `v2_forest` world; alternate calibrated images describe that same world.
Selecting/uploading an image does not replace Gazebo geometry. The world's
existing ground station is fixed in deploy mode and is not moved by robot setup.

Ownership is persisted in `/tmp/rs1-deploy-<uid>-<domain>/deployment.json`, separate
from `data_dir`, so changing backend data directories cannot create a duplicate
deployment. Keep that journal while its deployment is active. Plans and ROS logs
are saved under the deployment's backend data directory in `sessions/<id>/`,
including `plan.json` and `ros.log`. The default data directory is
`.mission-control/deploy`.

## Robot telemetry

The **Telemetry** section embeds a local, unmodified Lichtblick workspace for
maps, global TF, both robot cameras, Parrot terrain-following plots and mission
status. The normal `dashboard.launch.py` command starts the viewer and read-only
ROS bridge automatically in every dashboard mode. The normal setup installs
their dependencies; no additional telemetry command or flag is needed.

Use **Reset to starter layout** to apply the project's topic configuration.
Telemetry can remain open before and after robot spawning. Mission controls
stay in Mission Control; the bridge permits observation only. Stopping the
dashboard launch stops its viewer and bridge; browser refreshes do not.

If telemetry should survive a dashboard restart, opt out in the dashboard and
run telemetry in a separate sourced terminal:

```bash
# Dashboard terminal
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy telemetry:=false

# Separate telemetry terminal, in the same ROS domain
ros2 launch rs1_dashboard telemetry.launch.py
```

Also use `telemetry:=false` when viewer and bridge services are already running
elsewhere. See [telemetry setup, layout and remote connections](telemetry/README.md)
for ports, URL overrides and customisation.

## External mode: manual debugging with separate ROS launches

Normal simulation startup is unchanged and works without any dashboard:

```bash
# Terminal 1: complete world, robots, localisation, navigation and alignment
ros2 launch rs1_sim rs1_sim.launch.py

# Terminal 2: mission coordinator/executors, when needed
ros2 launch rs1_mission rs1_mission.launch.py

# Terminal 3: optional monitoring and mission controls
ros2 launch rs1_dashboard dashboard.launch.py mode:=external
```

`external` remains the dashboard launch's default.

1. Wait for the running simulation and mission coordinator to appear. The
   dashboard reports alignment, localization and navigation readiness.
2. Select a point ROI on the registered forest image, drag its marker, or edit
   the coordinates. The initial target comes from the mission coordinator when
   configured in the world frame. Only `v2_forest` and explicitly registered
   images of that world are currently supported by the dashboard.
3. Select **Start Mission**. An edited ROI is validated and applied atomically
   to the idle coordinator before `/mission/start`. An untouched target preserves
   the coordinator's configured frame and Z. The dashboard does
   not launch or respawn simulation or mission nodes. Follow live poses and
   **Robots & status**.
4. **Cancel Mission** requests cancellation and waits for executor results. It
   is not an emergency stop.
5. **Clear trails** clears dashboard trails and events only. Simulation, TF
   registration and mission state stay in their owning terminals.

Robot starts, spawn heights and ground-station placement belong to the simulation
launch in this mode. Refreshing or closing the browser, or stopping and restarting
the dashboard launch, leaves the separately launched simulation and mission
processes running. The dashboard launch restarts its own telemetry viewer and
bridge. On reconnect, ROS status is authoritative. A restarted dashboard starts
recording new trails.

After completion, cancellation or failure, restart `rs1_mission.launch.py` in its
terminal to run another mission. When restarting/resetting the simulation, also
restart the mission launch so its TF buffers and goals belong to the new world.
Use the same `session_id` if overriding its `manual` default in both launches.

## Optional managed mode: plan and launch from the dashboard

In an unused ROS domain, start only:

```bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=managed
```

Do not also start the simulation and mission terminals in this mode. The backend
owns both through the saved-plan launch pipeline:

1. Select **Forest overhead**, or upload an explicitly calibrated image in
   **Settings**. Only the existing `v2_forest` world is supported initially.
2. Select **Robot start**, then click a clearing. One point sets both robots' XY.
3. Select **Point ROI**, then click the traversal target. Drag markers or edit
   numeric fields. The station initially follows a `(0, +5)` m offset from start.
4. Review terrain validation and spawn heights. Use pan, zoom and fit controls.
5. Select **Start Mission**. The backend saves the plan, generates a world copy,
   launches the simulation, registers both maps, checks navigation, and calls
   `/mission/start`. Follow details in **Robots & status**.
6. **Cancel Mission** requests cancellation and waits for executor results. It is
   not an emergency stop. Cancelling preparation shuts down the owned launch.
7. **New session** stops the owned simulation and resets TF buffers, alignment,
   mission state and trails before accepting another plan. Reset is also required
   after a map reset, respawn, completion or failure. Saved logs remain available.

Refreshing, closing or reconnecting a browser only reads backend state. It cannot
launch a second simulation or repeat a registration. Concurrent starts return
HTTP 409. Plan edits are locked once a session is accepted. A second backend in
the same ROS domain is rejected with a filesystem lock; an already occupied ROS
graph is also rejected before launching. Run CLI probes in the same domain and
with the same `ROS_LOCALHOST_ONLY` / RMW settings as the backend.
Shutting down the backend in managed mode stops its owned simulation. Use deploy
mode when the world and robots should survive a dashboard backend restart.

## Coordinates and terrain

The preset is defined in `rs1_ws/src/rs1_sim/config/map_presets.json`. The default
image is the existing 1024 × 1024 `forest_overhead.png`. The forest model is at
world origin with no rotation. Its collision mesh spans `[-100, 100]` m on both
axes. The metadata and the OBJ's texture coordinates confirm image-down is +Y.

```text
world_x = x_min + u * (x_max - x_min)
world_y = y_min + v * (y_max - y_min)
u = (world_x - x_min) / (x_max - x_min)
v = (world_y - y_min) / (y_max - y_min)
```

`u,v` are continuous fractions across the **displayed image edges**. Thus the
top-left outer edge is (-100,-100), bottom-right is (100,100), and centre is
(0,0). A raster pixel centre at column `i` corresponds to `(i+0.5)/1024` in this
UI convention. Do not divide a UI edge position by 1023. The source metadata's
first/last *sample* mapping and the DEM's 257 grid vertices are separate
sample-centre conventions. The water-mask lookup intentionally uses nearest
samples and `(width-1)`, unlike continuous image display coordinates.

The SVG map uses a world-coordinate viewBox; pointer conversion uses the inverse
screen transformation matrix. Zoom, pan, letterboxing and resize therefore do
not change world coordinates. SVG image bounds, markers, headings and metre
scale all share the same calibration. With +Y down, positive world yaw appears
clockwise on screen; yaw zero points right (+X).

Uploads require their own image/calibration IDs, explicit axis-aligned world
edge bounds and an operator registration acknowledgement. Matching image
dimensions never imply registration. Uploads do **not** replace Gazebo geometry.
Rotated or mirrored images must be registered externally before upload.

For managed and deploy placement, elevation is barycentrically interpolated from
the **binary STL collision triangles**, with a spatial index. Husky spawn uses the highest sampled height
under its footprint + 0.13228 m wheel-to-base extent + 0.10 m initial clearance.
Parrot starts at ground + configured sensor clearance + 0.10 m sensor offset;
default clearance is 4 m, matching the existing terrain-following target.
Changing the initial clearance does not change that controller's target.
In managed mode, station height uses its own sampled footprint + 0.05 m. The
generated SDF moves the single existing station; it does not append another
station. Deploy mode leaves the already-loaded station unchanged.

Validation rejects nonfinite coordinates, out-of-bounds footprints, water,
proximity to tree/rock/bush origins, and station/robot clearance. Managed mode
requires at least 4 m between its selected station and shared start; deploy mode
requires that clearance from the world's existing station without moving it.
These conservative checks identify obvious placement errors; they are not a
full canopy/mesh collision or traversability planner. Review the Gazebo scene
for unusual placements or custom images.

## TF ownership and registration

```text
world ─ husky1_map ─ husky1_odom ─ husky1_base_link ─ sensors
      └ parrot1_map ─ parrot1_odom ─ parrot1_base_link ─ sensors
```

There is no mission frame. SLAM retains ownership of map→odom, the EKF retains
odom→base, and robot_state_publisher retains the sensor chain. Existing Nav2 and
localization nodes continue to use their namespaced TF topics.

`rs1_alignment` subscribes to each robot's `/tf` and `/tf_static` **once**,
aggregates them one-way onto global `/tf` and `/tf_static`, and adds only
world→map registrations. It republishes the accumulated static set with
transient-local durability for later subscribers. There is no relay back into
robot namespaces. Mission executors, the coordinator, perception, RViz and backend
consume global TF. Start this component only once per session.

Normal `rs1_sim.launch.py` starts alignment with the robot stack. With `ui:=true`
that stack is skipped; deploy mode starts alignment later through the reusable
robot bringup launch. The mission-only launch and external dashboard do not start
a second copy. Set `alignment:=False` only when another custom normal launch
owns it. Restarting a deploy or external dashboard leaves alignment unchanged.

The obsolete perception `parrot_world_broadcaster.py` and its launch/install
references have been removed. No direct world→base truth transform is published.
Perception now requires the aligned localization tree; teleport-based perception
accuracy tests cannot assume that localization instantly follows Gazebo truth.

Gazebo PosePublisher emits stamped model poses, bridged only to
`/mission/truth/{husky1,parrot1}` as TFMessage **data**, not as TF broadcasts.
Fortress needs both `publish_model_pose` and `publish_nested_model_pose` true;
see its [PosePublisher implementation](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.cc).
Both URDF roots are base_link, so the current model→base offset is zero. The
alignment node exposes `{robot}_model_to_base: [x,y,yaw]` for future models with a
different origin. Update both these offsets and spawn-height geometry when
changing robot descriptions.

For each robot, the alignment component looks up map→base at the truth message's
**exact simulation timestamp**, retrying until historical TF is available:

```text
world_T_map = world_T_base(truth) × inverse(map_T_base(estimated))
```

It publishes the resulting XY translation and yaw as static TF. Nonzero local
positions and headings are handled; spawn translation is never blindly copied.
Ten matched samples must agree within **0.25 m XY and 5° yaw** before readiness.
`/mission/alignment` includes the initial verification samples and current truth
residuals. The transform is never continuously corrected to hide drift. The UI
poses and trails always come through estimated world→map→odom→base TF.

Registration is planar: world→map Z is **0**, with roll/pitch zero. Local map Z is
not a terrain-relative altitude measurement. In particular, Parrot's EKF does
not fuse its changing Z. The dashboard deliberately reports only XY/heading;
accurate 3D tree placement remains limited by vertical localization. Startup
does not wait for altitude settling. Terrain following remains active when its
downward sensor sees Husky's roof and as it subsequently sees the ground.

## Mission configuration, readiness and API

In deploy mode, `POST /api/deployment/preview` validates a spawn-only plan and
`POST /api/deployment/spawn` deploys it into the running `ui:=true` world:

```json
{
  "session_id": "8df831b3-f3bb-4b8d-a9e8-0d3ad628c631",
  "world_id": "v2_forest",
  "image_id": "forest-overhead",
  "calibration_id": "forest-edges-v1",
  "start": {"x": 90, "y": -48},
  "parrot_clearance": 4
}
```

There is no ROI or station field in a deployment request. The backend checks the
world identity, ROS graph and Gazebo entities to prevent duplicate robots before
starting `rs1_mission/deploy.launch.py`. The response's `deployment` object tracks
the saved plan, placements, preparation state and whether an ROI was selected.
`POST /api/deployment/stop` with `{}` stops that deployment after mission cleanup
and removes its models; it never stops the operator's world.

In deploy and external modes, `POST /api/mission/start` accepts a world ROI:

```json
{"roi": {"x": 88, "y": -48}}
```

The backend checks the selected point against the existing forest terrain and
sets the idle coordinator's world ROI parameters atomically before requesting
start. In external mode, sending `{}` uses the coordinator's configured target.
Deploy mode requires an explicit ROI for its first mission. Placement fields
are not accepted in either mode. The simulation and mission process instances are
tracked so a restarted simulation clears stale backend TF and trails, and a
restarted coordinator supplies fresh mission state. External mode cannot start,
stop, or reset external ROS processes; deploy mode controls only its recorded
robot and mission deployment.

In managed mode, the same endpoint accepts a validated plan:

```json
{
  "session_id": "8df831b3-f3bb-4b8d-a9e8-0d3ad628c631",
  "world_id": "v2_forest",
  "image_id": "forest-overhead",
  "calibration_id": "forest-edges-v1",
  "start": {"x": 90, "y": -48},
  "roi": {"x": 88, "y": -48},
  "station_offset": {"x": 0, "y": 5},
  "parrot_clearance": 4
}
```

The managed backend writes `sessions/<id>/plan.json`, including resolved spawn/station/ROI
heights, and `world.sdf`. The focused `mission_control.launch.py` reads this
immutable session configuration. The coordinator receives world ROI parameters
at construction, then uses tf2 to transform the ROI into `parrot1_map` before
submitting it to the existing executors. Husky retains its existing path
transformation into `husky1_map`; traversal algorithms are unchanged.

Starting a mission in any mode requires a running clock, both verified
registrations, fresh estimated base transforms, active
planner/controller/bt_navigator lifecycle nodes, the traversal/navigation/exploration
action servers, and mission start/cancel services. Existing short launch timers
are retained to avoid EKF spawn spikes;
they are not the mission readiness gate. Managed and deploy preparation wait up
to 180 s; external mode reports unavailable prerequisites while you bring up the
ROS stack. External and deploy modes accept the simulation's retained successful
initial verification when the dashboard connects later. Both still require fresh
TF/status and reject invalidated alignment; current truth residuals beyond the initial tolerance are
reported without re-registering the robot or replacing its estimated pose.

The coordinator publishes typed `rs1_interfaces/msg/MissionStatus` on
`/mission/status` with session ID, process instance ID, configured ROI, state,
robot stages, message and cleanup flag.
Final outcomes remain available instead of immediately returning to IDLE. The
backend tracks preparation and cancellation. In managed mode, loss of required
runtime interfaces for 8 s stops its owned simulation and records a failure;
cancellation requires executor completion within 30 s. Processes are stopped
only through the process group created by the backend (SIGINT, then bounded
TERM/KILL escalation). External mode reports ROS status and request failures
without terminating the operator's processes. Deploy mode uses the same mission
requests and leaves the world under its original launch. Its detached robot
launch and world guard continue operating without a dashboard backend.
An uncertain Start/Cancel response is not automatically retried.

Other endpoints: `GET /api/state`, `GET /api/presets`, `GET /api/images/<id>`,
`POST /api/images` (multipart file + registration JSON),
`POST /api/plan/preview`, `POST /api/mission/cancel`, and
`POST /api/session/reset` (clear trails/events in external and deploy modes; stop and reset
the owned simulation in managed mode). Interactive API schema: `/docs`.

Configuration environment variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `MISSION_MODE` | `external` in dashboard launch; `managed` with direct Uvicorn | `deploy` controls robots in a UI world; `external` attaches to operator-owned ROS; `managed` owns the whole simulation. |
| `ROS_DOMAIN_ID` | Inherited; launch uses `0` if unset | Must match the other ROS terminals. Direct Uvicorn retains default `42`. |
| `ROS_LOCALHOST_ONLY` | Inherited; launch uses `0` if unset | Must match the other ROS terminals. Direct Uvicorn retains default `1`. |
| `MISSION_REPO` | inferred checkout | Repository root containing terrain assets. |
| `MISSION_DATA` | Launch: `<repo>/.mission-control/external` for external, `…/deploy` for deploy, `<repo>/.mission-control` for managed | Images and backend data; deploy and managed sessions save plans and ROS logs. |
| `MISSION_PREPARE_TIMEOUT` | `180` | Managed/deploy preparation deadline in wall seconds. |
| `MISSION_HEADLESS` | `True` | Managed server-only Gazebo; set `False` for the scene GUI. |

The managed backend uses a unique `IGN_PARTITION` per session. `headless` suppresses the
Gazebo GUI but GPU sensor rendering still needs working graphics/EGL. Use the
existing container's display/GPU setup; inspect each session's `ros.log` if
clock or sensor readiness times out. In managed mode, no simulation or process
is adopted after a backend crash: saved active state becomes failed, and an occupied ROS graph
prevents duplicate starts. If the backend was killed without shutdown, stop its
orphaned launch explicitly before resetting; never kill unrelated ROS processes.

Standalone simulation spawn arguments now actually work:

```bash
ros2 launch rs1_sim rs1_sim.launch.py \
  husky_x:=90 husky_y:=-48 husky_z:=4.272 husky_yaw:=0 \
  parrot_x:=90 parrot_y:=-48 parrot_z:=8.113 parrot_yaw:=0
```

These direct launch arguments are model-origin poses. Managed and deploy dashboard
modes calculate robot heights; only managed mode generates a moved station. `world_file`
and `headless` are also supported launch arguments. Alignment and shared
cross-robot TF already start with this launch; do not run another alignment node.

## Validation

```bash
# Backend math, terrain, lifecycle and HTTP/upload tests (no running ROS needed).
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/mission-control/bin/python -m pytest \
  mission_control/backend/tests -q

source .venv/frontend-tools/env.sh
cd mission_control/frontend
pnpm test
pnpm build
cd ../..
```

Disabling plugin autoload isolates these tests from globally installed ROS
`launch_testing` plugins, whose pytest version requirements may differ.

For a managed-mode end-to-end smoke test, start the dashboard in managed mode
with a sourced build and use
start `(90,-48)`, station offset `(0,5)` and ROI `(88,-48)`. Wait for traversal
completion. In another terminal with the **same ROS domain and middleware
settings**, run this read-only late-subscriber probe:

```bash
source rs1_ws/install-mission-control/setup.bash
# Match the domain and middleware settings of the running ROS stack.
.venv/mission-control/bin/python mission_control/backend/tests/ros_probe.py
```

It checks the connected tree, single TF parents, timestamp-matched truth errors
and a world→Parrot-map→world ROI round trip. Run it after starting a session;
an already-running probe intentionally prevents managed mode's empty-domain check.
Then test refresh/reconnect, duplicate start, cancellation on a longer traversal,
and New session. Reset must clear trails and registrations and remove only the
owned simulation. See `VALIDATION.md` for the checks performed during delivery.

For external mode, follow [the three-terminal workflow](LAUNCHING.md), run the
same read-only probe, and check that dashboard shutdown leaves ROS running,
reconnection shows the current mission, and Clear trails leaves ROS state intact.

For deploy mode, follow [the two-terminal workflow](LAUNCHING.md), spawn before
selecting an ROI, then start a short traversal. Check browser and backend restart
without duplicate spawning, Cancel followed by Stop robots, and a fresh deployment
after restarting the world. The UI must block respawning in the old world instance.
Stopping the world must stop the detached robot stack through
its world guard. These are verification instructions; recorded run results belong
in `VALIDATION.md`.

Deferred: polygon ROI, live sensor viewers, tree sampling, return-home,
biomass/results, arbitrary worlds, automatic image registration and full 3D
localization. Simulation setup and ROS transport are separate backend modules
so future hardware adapters do not require changing map interaction code.
