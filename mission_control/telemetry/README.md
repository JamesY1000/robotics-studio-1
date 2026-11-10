# Robot telemetry with Lichtblick

Mission Control's **Telemetry** section embeds the unmodified upstream
Lichtblick web application. The dashboard retains mission and deployment
controls; Lichtblick displays ROS topics through a separate Foxglove WebSocket
bridge. The bridge disables client publishing, service calls and parameter
changes. Opening, closing or reloading the viewer does not spawn robots or
change a mission.

## Install and launch

Inside the devcontainer, the normal repository setup installs the ROS bridge and
downloads the official **Lichtblick v1.29.1** web release:

```bash
bash .devcontainer/scripts/setup_dependencies.sh
```

Rebuild `rs1_dashboard` through your usual colcon build and source the matching
workspace install. Start the dashboard normally; it includes the viewer and
read-only bridge automatically. For the deploy workflow:

```bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy
```

The launch serves the dashboard at `http://127.0.0.1:8000`, the viewer at
`http://127.0.0.1:8080` and the bridge at `ws://127.0.0.1:8765`. No extra telemetry
flag or terminal is needed. All dashboard modes include telemetry; `external`
remains the default mode when `mode` is omitted. Telemetry starts before robot
deployment. The viewer
has no robot data until the simulation's robot publishers are running. The
dashboard's bridge badge reports ROS node discovery, while the viewer reports
its actual WebSocket connection and received data.

The installer verifies pinned SHA256 checksums and stores the unmodified assets
under `.venv/lichtblick/1.29.1`, with the upstream license and source reference.
Its file manifest permits an intact installation to be reused offline. To
reinstall only the viewer assets, run
`bash .devcontainer/scripts/setup_telemetry_viewer.sh`; no Node build, Docker
daemon or extra Python packages are required for the viewer server.

Stopping the dashboard launch also stops its viewer server and bridge. Browser
refreshes leave all services running. In deploy and external modes, the simulation
and robot/mission stacks retain their existing ownership and keep running when
the dashboard launch stops.

If telemetry should survive a dashboard restart, opt out of the integrated
services and start them in a separate sourced terminal:

```bash
# Dashboard terminal
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy telemetry:=false

# Separate telemetry terminal, in the same ROS domain
ros2 launch rs1_dashboard telemetry.launch.py
```

The standalone launch owns only the viewer server and bridge; stopping it leaves
the simulation, robots, mission and dashboard running. Also use
`telemetry:=false` when viewer and bridge services already run elsewhere. Run
only one telemetry launch per ROS domain.

## Starter workspace

Use **Reset to starter layout** in the Telemetry section, or download the starter
JSON and import it through Lichtblick's Layout menu. The checked-in
[layout.json](layout.json) contains the world and camera panels,
with **Mission & commands**, **ROS logs**, and **Exploration** tabs underneath:

| Panel | Actual ROS data | Meaning |
| --- | --- | --- |
| World | `/tf`, `/tf_static`, both robots' `/robot_description`, `/map`, `/plan`, `/local_plan`, `/husky1/scan_filtered`, `/parrot1/scan`, `/parrot1/downwards_scan`, `/parrot1/explore/frontiers`, `/mission/roi_marker` | Actual URDF robot models, registered SLAM maps, navigation scans, planned paths, frontier candidates, robot frame axes and ROI in `world`. Husky overlays are cyan; Parrot overlays are amber. |
| Husky camera | `/husky1/camera/image/compressed` | JPEG from the Husky RGB camera. |
| Parrot camera | `/parrot1/camera/image/compressed` | JPEG from the Parrot RGB camera. |
| Downward centre ray | `/parrot1/downwards_scan.ranges[1]` | The same centre ray used by terrain following, in metres. |
| Selected commands | `/parrot1/cmd_vel_controlled.linear.z`, `/parrot1/cmd_vel_controlled.linear.x`, `/husky1/cmd_vel_controlled.linear.x` | Final vertical/forward commands sent to Gazebo after autonomous/manual selection, in m/s; these are not measured vehicle velocities. |
| Mission status | `/mission/status` | The existing `rs1_interfaces/msg/MissionStatus`, including state, robot stages, ROI, coordinator message, control mode and selected manual input. |
| ROS logs | `/rosout` | ROS node logging with logger-name, severity and text filters; not a mirror of a particular terminal. |
| Exploration | `/parrot1/explore/status` | Guided exploration's current status. |

For a saved layout, select the corresponding `/camera/image/compressed` topic in
each robot's Image panel, or use **Reset to starter layout**. Perception can
continue subscribing to the raw `/husky1/camera/image` and `/parrot1/camera/image`
topics. The bridge encodes JPEGs when the compressed topic has subscribers.

Husky's raw `/scan` still exists, but SLAM and Nav2 use `/scan_filtered` after
terrain filtering. Likewise, Parrot's `/cmd_vel_combined` still exists as the
autonomous input to the selector; `/cmd_vel_controlled` shows its final decision
during both autonomy and manual assistance.

The starter layout hides both `/camera/depth/points` layers to keep the RGB
panels responsive during manual driving. Enable either robot's depth cloud in
the 3D panel when inspecting geometry. Each cloud uses a separate stream.

Both RGB-D sensors default to 6 Hz. For smoother manual driving, start the
dashboard with `RS1_CAMERA_HZ=15 ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy`
before deploying into a fresh world. For standalone robot bringup, pass
`camera_hz:=15` to `rs1_sim.launch.py`. This setting takes effect at spawn.
The Phase 2 sweep publishes live tree estimates on `/parrot1/tree_map_markers`, enabled in the 3D panel. Orange spheres mark oak estimates and green spheres mark pines. The final sweep result replaces the candidates. The separate `/tree_locations/*` debug layers require the perception visualisation scripts.

The bridge also exposes `/teleop/joy`, `/teleop/cmd_vel_controller`, and
`/yolo/debug_image` for custom Raw Messages, Plot or Image panels. The YOLO image
requires the perception node. Adding a topic to the bridge's allowlist does not
start that publisher or make the viewer subscribe to it automatically.

The world panel starts with an overhead view centred on the forest. Zoom and pan
to your deployment, or change the display frame/camera in the panel settings.
It uses the existing world-to-map alignment; it does not create transforms or
overlay Gazebo ground truth. Missing `world`/robot frames before alignment and
empty camera panels before spawning are expected. ROI markers appear when the
coordinator publishes them. No camera calibration topic is assumed: the current
simulation bridge exposes image topics without a CameraInfo topic.

The **Husky robot model** and **Parrot robot model** custom layers load the live
`/husky1/robot_description` and `/parrot1/robot_description` topics. Their URDF
links already have robot-specific prefixes, so the layers use an empty frame
prefix. Visual meshes and materials come from `rs1_description` through the
bridge's existing asset support using `package://` URIs. Gazebo resolves the same
URIs using the resource path exported by `rs1_description/package.xml`.
After rebuilding a mesh-URI change, restart the simulation and robot publishers
so they publish the updated description and Gazebo picks up its resource path.
The model layers need the description topics and aligned TF before they appear.

The range plot is a raw beam measurement. Robot tilt and invalid/out-of-range
returns affect it; it is not an independent, filtered ground-clearance estimate.
No reference line is hardcoded because the terrain controller's target is
configurable. Plots use receive time so stamped scans and unstamped commands
share a timeline. These are display measurements; the ROS controller remains
authoritative.

Save and export your own layouts after customising them. **Reset to starter layout**
explicitly reloads the repository preset; ordinary viewer reloads should retain
the current saved workspace. Upstream's `layoutUrl` importer replaces existing
layouts with the same URL-derived name, so duplicate or export a customised
starter before loading the preset again. Browser layouts live in IndexedDB and
can be lost when browser site data is cleared.

## Log monitoring

Open the **ROS logs** tab in the preset. In the Log panel settings, **Namespaces**
lists the logger names seen so far. Use **Hide all**, then enable the loggers you
want, or use the severity and text filters. Newly encountered logger names are
visible by default, so this is a display filter rather than a fixed launch boundary.
The panel can only show levels that the source nodes actually publish; selecting
Debug in the panel does not change a node launched with `log_level:=info` or
`stack_log_level:=warn`.

`/rosout` contains ROS log messages from nodes in the ROS domain, regardless of
which terminal started them. It has no terminal number or launch-session field.
Plain stdout/stderr, Python prints, Uvicorn messages and ROS launch announcements
are not automatically published to `/rosout`.

For `ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy`, the dashboard
already tails the deployment's `ros.log` into its terminal when `show_logs:=true`.
Those ROS nodes also normally publish their own log messages to `/rosout`, so
they can be monitored in Lichtblick. The terminal additionally contains backend,
viewer and launch output that the ROS Log panel cannot reproduce.

Showing **all and only that dashboard launch's terminal output** would require
a dedicated stdout/stderr capture relay publishing `rcl_interfaces/msg/Log` on
a separate topic. No such terminal relay is installed by this preset update.
Use the terminal or deployment's saved `ros.log` when the complete process output
is needed; `/rosout` filters cannot provide that guarantee.

## Remote containers and existing viewers

The browser must be able to reach dashboard port **8000**, viewer port **8080**,
and bridge port **8765**. Forward all three when the devcontainer is remote.
Local launch defaults bind to loopback. On `dashboard.launch.py`,
`telemetry_host` sets the bind address for both integrated telemetry services.
On standalone `telemetry.launch.py`, use `viewer_host` and `bridge_host`
individually. Change these only when your chosen forwarding or trusted network
setup requires a different bind address.

The dashboard normally builds viewer and bridge URLs from the browser's
hostname. For viewer and bridge services already hosted elsewhere, opt out of
the local services and configure browser-reachable URLs on the dashboard process:

```bash
MISSION_TELEMETRY_VIEWER_URL=https://viewer.example.edu \
MISSION_TELEMETRY_BRIDGE_URL=wss://bridge.example.edu \
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy telemetry:=false
```

These are example hostnames, not provided services. If a reverse proxy forwards
to the services started by the dashboard itself, use the same URL overrides and
omit `telemetry:=false`. HTTPS dashboards need an
HTTPS viewer and a WSS bridge; arrange TLS through your existing proxy. A viewer
that refuses framing can be opened using **Open in new tab**. If automatic
layout loading is blocked by browser/network policy, download and import the
JSON manually. Reachability, TLS and framing permissions are separate from ROS
node discovery.

Both launch files accept `viewer_port`, `bridge_port`, and `repo_dir`. The
dashboard passes its configured ports to the integrated services and frontend.
For standalone services on different ports, set matching
`MISSION_TELEMETRY_VIEWER_PORT`/`MISSION_TELEMETRY_BRIDGE_PORT` on the dashboard,
or use the explicit URLs above.

Standalone `telemetry.launch.py` also accepts `viewer:=false` or `bridge:=false`
to start only one service. With an existing Lichtblick installation or the
official Foxglove desktop app, disable integrated telemetry on the dashboard
and run `ros2 launch rs1_dashboard telemetry.launch.py viewer:=false` for a
bridge only. Open a **Foxglove WebSocket** connection to that bridge. The
supplied preset targets the pinned Lichtblick version; Foxglove layout import
compatibility is not assumed.

## Upstream strategy and extension points

Use the pinned upstream release and keep project-specific JSON layouts and
future extensions here. Lichtblick is useful for a self-hosted lab or offline
field workflow. Foxglove's current official app remains an alternative for
teams that prefer its product/support and collaboration features. A project
fork of either viewer is unnecessary for this telemetry integration.

For new sampling telemetry, first publish appropriate ROS messages and add
panels to the layout. Use a Lichtblick message converter or custom panel when
the built-in panels cannot represent the data. Only consider changing the core
viewer when a concrete requirement exceeds those extension APIs. Test the
viewer and bridge together before upgrading the pinned release.

The layout was checked against official v1.29.1 source and this repository's
topic definitions:

- [Layout example](https://github.com/lichtblick-suite/lichtblick/blob/v1.29.1/e2e/fixtures/assets/layout1.json), [layout validation and panel title key](https://github.com/lichtblick-suite/lichtblick/blob/v1.29.1/packages/suite-base/src/util/layout.ts).
- [Image/3D configuration](https://github.com/lichtblick-suite/lichtblick/blob/v1.29.1/packages/suite-base/src/panels/ThreeDeeRender/IRenderer.ts), [plot configuration](https://github.com/lichtblick-suite/lichtblick/blob/v1.29.1/packages/suite-base/src/panels/Plot/utils/config.ts).
- [URL state](https://github.com/lichtblick-suite/lichtblick/blob/v1.29.1/packages/suite-base/src/util/appURLState.ts) accepts `ds=foxglove-websocket`, `ds.url=<ws-or-wss-url>` and `layoutUrl=<absolute-http-or-https-json-url>`; [the importer](https://github.com/lichtblick-suite/lichtblick/blob/v1.29.1/packages/suite-base/src/Workspace.tsx) controls replacement behavior. Cross-origin layout fetches require CORS; the dashboard's layout endpoint provides it for this public preset.
- [ROS 2 connections](https://lichtblick-suite.github.io/docs/docs/connecting-to-data/frameworks/ros2/), [layouts](https://lichtblick-suite.github.io/docs/docs/visualization/layouts/), [extensions](https://lichtblick-suite.github.io/docs/docs/extensions/introduction/), and [upstream releases](https://github.com/lichtblick-suite/lichtblick/releases).
