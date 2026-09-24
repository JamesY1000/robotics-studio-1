# Mission control dashboard (mission_control_gui + mission_interfaces)

Two packages:

- **`mission_interfaces`** - custom `TreeSample`/`TreeSampleArray` messages
  (id, species, DBH, sampled/pending status, position) used to drive the
  Selected Trees table.
- **`mission_control_gui`** - the standalone Qt (C++) dashboard itself.
  Still a plain executable (no `pluginlib`, no plugin discovery) - pumps
  `rclcpp::spin_some()` off a `QTimer` every 20 ms, same pattern as
  before.

## Layout (matches the reference design)

- **Sidebar** (left): branding, nav list (only "Mission" is implemented -
  the others are shown disabled for layout parity), and a live System
  Status panel (UAV/UGV/Comms dots).
- **Top bar**: title, clock, a READY/NOT READY badge, and Start
  Mission / Emergency Stop / Return Home buttons that act on *all*
  robots at once.
- **Map card**: AGB heat-map, ROI boundary (dashed white polygon), a
  fixed ground-station icon, dashed robot path trails, gray "other
  tree" markers, green "selected tree" markers, a compass, a scale
  bar, and zoom (+/-/recenter buttons, scroll wheel) + click-drag pan.
  The "Traversability"/"Selected Trees"/"Robot Paths" view-toggle
  buttons are shown but disabled - this build always renders every
  layer together rather than switching between them.
- **Mission Progress card**: progress bar + a 5-step checklist driven
  by a single `/mission/stage` topic (simpler and more idiomatic than
  five separate booleans).
- **Selected Trees card**: a table of id/species/DBH/status from
  `/selected_trees`.
- **AGB Estimate card**: current estimate + delta vs. initial map.
  "View Full Results" is a disabled placeholder.
- **Robot cards** (right column, Parrot UAV then Husky UGV): online
  status, battery, an RGB camera feed, a LiDAR-view placeholder (see
  note below), and telemetry (altitude for the UAV, position for the
  UGV; speed and mode for both).

## What's simplified vs. the reference design (and why)

- **LiDAR view is a placeholder**, not a rendered point cloud.
  Rendering `sensor_msgs/PointCloud2` properly needs either an OpenGL
  widget or careful binary field parsing - both are easy to get subtly
  wrong without a way to test against real hardware, so it's left as a
  clearly-labeled placeholder rather than shipping something that
  might silently misrender. See "Extending this" below for how to add
  it.
- **Zoom controls sit beside the map**, not overlaid on its top-right
  corner - simpler and more robust than manually positioning floating
  buttons on top of a custom-painted widget. Scroll-wheel zoom and
  click-drag pan work directly on the map either way.
- **The map-layer toggle buttons and other sidebar nav items are
  disabled placeholders.** This build always shows every map layer at
  once and only implements the "Mission" page - toggling between
  layers/pages isn't wired up.
- **Mission progress uses one `/mission/stage` string topic** instead
  of five separate booleans - simpler to publish from a real mission
  state machine and to derive the checklist icons from.

## IMPORTANT SAFETY NOTE

The Emergency Stop button is a software, network-based e-stop -
suitable for simulation and demos, but **not a certified safety stop
for real hardware.** It depends on the network link being up and each
robot's driver actually subscribing to and acting on `/e_stop`. For
real Husky/UAV hardware, use the platform's physical hardware e-stop
as the actual safety mechanism, and treat this GUI's button as a
convenience/secondary control layered on top of it, not a replacement.

## Topics

| Purpose | Topic | Type | Package |
|---|---|---|---|
| Velocity command | `/{robot}/cmd_vel` | `geometry_msgs/Twist` | core |
| E-stop signal | `/{robot}/e_stop` | `std_msgs/Bool` | core |
| Robot pose (+ speed via twist) | `/{robot}/odom` | `nav_msgs/Odometry` | core |
| Return-to-home trigger | `/{robot}/return_home` | `std_msgs/Bool` | core |
| Battery level | `/{robot}/battery_percent` | `std_msgs/Float32` (0-100) | placeholder |
| Current mode text | `/{robot}/mode` | `std_msgs/String` | placeholder |
| RGB camera feed | `/{robot}/camera/image_raw` | `sensor_msgs/Image` (`rgb8`/`bgr8` only) | placeholder |
| AGB map | `/agb_map` | `nav_msgs/OccupancyGrid` (0-100 = biomass) | placeholder |
| Other detected trees (gray) | `/tree_detections` | `visualization_msgs/MarkerArray` | placeholder |
| Selected/sampled trees (green + table) | `/selected_trees` | `mission_interfaces/TreeSampleArray` | placeholder |
| ROI boundary | `/roi_boundary` | `geometry_msgs/PolygonStamped` | placeholder |
| Comms health | `/system/comms_ok` | `std_msgs/Bool` | placeholder |
| Mission stage | `/mission/stage` | `std_msgs/String` (`uav_survey`\|`traversability`\|`sampling`\|`returning`\|`complete`) | placeholder |
| Overall mission progress | `/mission/progress` | `std_msgs/Float32` (0-100) | placeholder |
| Trees sampled / total | `/mission/trees_sampled`, `/mission/trees_total` | `std_msgs/Int32` | placeholder |
| AGB estimate + delta | `/agb_estimate/current_t_per_ha`, `/agb_estimate/delta_percent` | `std_msgs/Float32` | placeholder |

`{robot}` is `husky1` or `parrot1`.

## Build

```bash
cd ~/ros2_ws/src
# copy/clone BOTH mission_interfaces and mission_control_gui here
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mission_interfaces mission_control_gui
source install/setup.bash
```

`mission_interfaces` must build before (or alongside) `mission_control_gui`
- `colcon build` figures this out automatically from the `<depend>` in
`mission_control_gui/package.xml`, but if you ever build with
`--packages-select mission_control_gui` alone on a fresh workspace,
build `mission_interfaces` first.

## Run

```bash
source install/setup.zsh
ros2 run rs1_mission_control_gui rs1_mission_control_gui
```

## Testing without real robots

```bash
# Robot online + position (watch the UAV/UGV dot in the sidebar go green)
ros2 topic pub /parrot1/odom nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 5.0, y: 3.0, z: 42.0}}}}" -r 5
ros2 topic pub /husky1/odom nav_msgs/msg/Odometry \
  "{pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}" -r 5

ros2 topic pub /system/comms_ok std_msgs/msg/Bool "{data: true}" -r 1

# AGB map (small grid for a quick visual check)
ros2 topic pub /agb_map nav_msgs/msg/OccupancyGrid \
  "{info: {width: 5, height: 5, resolution: 4.0}, \
    data: [10,20,30,40,50, 20,30,40,50,60, 30,40,50,60,70, 40,50,60,70,80, 50,60,70,80,90]}" \
  -r 1

# ROI boundary
ros2 topic pub /roi_boundary geometry_msgs/msg/PolygonStamped \
  "{polygon: {points: [{x: -10, y: -10}, {x: 10, y: -10}, {x: 10, y: 10}, {x: -10, y: 10}]}}" \
  -r 1

# Trees
ros2 topic pub /tree_detections visualization_msgs/msg/MarkerArray \
  "{markers: [{pose: {position: {x: 2.0, y: 3.0}}}, {pose: {position: {x: -3.0, y: 1.0}}}]}" -r 1
ros2 topic pub /selected_trees mission_interfaces/msg/TreeSampleArray \
  "{trees: [{id: 'T01', species: 'Eucalyptus', dbh_cm: 32.6, status: 1, position: {x: 4.0, y: -2.0}}, \
            {id: 'T02', species: 'Acacia', status: 0, position: {x: -1.0, y: -4.0}}]}" -r 1

# Mission progress
ros2 topic pub /mission/stage std_msgs/msg/String "{data: 'sampling'}" -r 1
ros2 topic pub /mission/progress std_msgs/msg/Float32 "{data: 60.0}" -r 1
ros2 topic pub /mission/trees_sampled std_msgs/msg/Int32 "{data: 3}" -r 1
ros2 topic pub /mission/trees_total std_msgs/msg/Int32 "{data: 8}" -r 1

# AGB estimate
ros2 topic pub /agb_estimate/current_t_per_ha std_msgs/msg/Float32 "{data: 228.4}" -r 1
ros2 topic pub /agb_estimate/delta_percent std_msgs/msg/Float32 "{data: 12.0}" -r 1
```

Click **Emergency Stop** / **Return Home** and confirm with
`ros2 topic echo /husky1/e_stop` / `/husky1/return_home` in another
terminal.

## Extending this

- **Real LiDAR rendering**: replace the placeholder `QLabel` in
  `RobotCardWidget` with a small custom-painted widget that subscribes
  to `sensor_msgs/PointCloud2`, reads the `x`/`y`/`z` `float32` fields
  at their declared offsets, and draws each point as a dot colored by
  height (`z`) - similar in spirit to `MapWidget`'s `paintEvent`, but
  in 2D top-down or side projection rather than true 3D (true 3D would
  need `QOpenGLWidget`).
- **Mission-stage source of truth**: `/mission/stage` is a placeholder
  string. If you build a real mission state machine (e.g. a
  behavior tree or an action server), have it publish this topic (or
  swap it for an action/service-based status query) rather than
  inferring stage from other signals.
