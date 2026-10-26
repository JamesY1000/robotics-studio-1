# `rs1_sim`

`rs1_sim` brings up the Gazebo simulation environment for Robotics Studio 1. It
can load one of the included world SDFs, spawn the Husky and Parrot robots, bridge
their Gazebo interfaces to ROS 2, publish their robot descriptions and TF, and
open a separate RViz window for each enabled robot, and include each robot's
localisation, SLAM, and Nav2 launch files. It also owns the shared world/map
alignment and global TF aggregation node by default.

## Package responsibilities

| Launch file | Responsibility |
| --- | --- |
| `rs1_sim.launch.py` | Main entry point. Starts the complete simulation by default; `ui:=true` starts the world, clock bridge and world-session marker, with optional RViz. |
| `gz_world.launch.py` | Starts Gazebo with a selected world, bridges the Gazebo `/clock` topic to ROS 2, and starts the `rs1_safety` E-stop velocity gate. |
| `robot_bringup.launch.py` | Spawns the robots and starts their supporting ROS nodes in an existing world. Shared by normal simulation and dashboard deployment. |
| `rviz.launch.py` | Opens the existing Husky/Parrot RViz displays independently of robot spawning. |
| `spawn_husky.launch.py` | Publishes the Husky description and TF, starts its Gazebo bridge, and creates the `husky1` model. |
| `spawn_parrot.launch.py` | Publishes the Parrot description and TF, starts its Gazebo bridge, and creates the `parrot1` model. |

The [Mission Control dashboard](../../../mission_control/README.md) can run in
its own terminal and attach to this simulation and a separately launched
`rs1_mission`. It gates traversal on verified world alignment and navigation
readiness. See [the three-terminal workflow](../../../mission_control/LAUNCHING.md).
The optional managed dashboard mode additionally computes terrain-aware spawn
heights and moves the single ground station in a generated world. The standalone
defaults below preserve the previous fixed spawn poses; use explicit Z values
or dashboard deployment for terrain-aware placement.

For click-to-spawn demos, launch the world with `ui:=true`, then launch
`rs1_dashboard dashboard.launch.py mode:=deploy` in another terminal. The
dashboard deploys robot bringup and the mission nodes into that existing world.
World selection stays with the simulation launch; the dashboard image must
match that world. Deployment currently supports the baseline `v2_forest` world,
without `world_file` overrides. The existing ground station stays in place.

Robot descriptions and meshes are provided by `rs1_description`. The Gazebo
worlds, simulation models, bridge configurations, and RViz configurations are
stored in this package.

## Supported environment

This package is developed against the project devcontainer's ROS 2 Humble and
Ignition Gazebo Fortress installation. Other ROS/Gazebo combinations have not
been tested. Gazebo and RViz are GUI applications, so a working display or X11
forwarding setup is required when running the default launch command in a
container.

## Build and source

From the repository root:

```bash
cd rs1_ws
colcon build --packages-up-to rs1_sim --symlink-install
source install/setup.zsh
```

For Bash, source `install/setup.bash` instead.

## Quick start

Start the default `v2_forest` world, both robots, both SLAM and Nav2 instances,
both RViz windows, and shared world/map alignment:

```bash
ros2 launch rs1_sim rs1_sim.launch.py
```

In separate terminals, the mission and dashboard can then be started with
`ros2 launch rs1_mission rs1_mission.launch.py` and
`ros2 launch rs1_dashboard dashboard.launch.py`. The dashboard does not own this
simulation. Do not start another alignment node alongside the default launch.

For the dashboard deployment workflow instead:

```bash
# Terminal 1: no robot models, robot nodes, alignment, or mission nodes yet.
ros2 launch rs1_sim rs1_sim.launch.py ui:=true

# Terminal 2: select a start position, deploy robots, select ROI, start mission.
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy
```

To open the Husky and Parrot RViz displays while waiting for dashboard deployment,
add `rviz:=true` to terminal 1:

```bash
ros2 launch rs1_sim rs1_sim.launch.py ui:=true rviz:=true
```

Their robot, TF, map and sensor displays populate as the deployed nodes start
publishing. Missing-frame/topic warnings before deployment are expected. RViz
belongs to the simulation terminal and stays open across dashboard restarts;
deployment does not open duplicate windows. `ui:=true` alone keeps RViz off.

Do not separately launch `rs1_mission` in this workflow: deployment launches it
once and leaves it waiting for Start Mission. The dashboard can be restarted
without stopping the deployed ROS launch. Closing Gazebo or its simulation
launch ends the world-session heartbeat; the deployment guard then shuts down
its robot and mission nodes within approximately ten seconds.

After **Stop robots**, restart the world terminal before deploying again. The
installed Fortress camera sensors can crash Gazebo when recreated in the same
world, so the dashboard blocks that operation. The dashboard can stay open
while you restart the world.

Launch arguments use the ROS 2 syntax `argument_name:=value`. To see the
arguments exposed by the installed launch file:

```bash
ros2 launch rs1_sim rs1_sim.launch.py --show-args
```

## Main launch arguments

These are the public arguments for `rs1_sim.launch.py`.

| Argument | Type / allowed values | Default | Purpose |
| --- | --- | --- | --- |
| `ui` | Boolean | `False` | Start Gazebo, its clock bridge and the world-session marker; the dashboard deploys robots later. RViz can be enabled separately. Robot-selection arguments only select displays in this mode. |
| `world` | `large_demo`, `simple_trees`, `v1_forest`, or `v2_forest` | `v2_forest` | Gazebo world to load. The `.sdf` extension is added automatically. |
| `world_file` | SDF path | empty | Optional generated world file; overrides the selected world asset. |
| `headless` | Boolean | `False` | Run Gazebo server without GUI (sensor rendering still required). |
| `alignment` | Boolean | `True` | Start one shared world/map registration and global TF aggregator. Disable only if another custom launch owns it. |
| `session_id` | String | `manual` | Session label; match the mission launch when overriding. |
| `use_sim_time` | Boolean | `True` | Makes ROS nodes use the Gazebo `/clock` simulation time. |
| `husky` | Boolean | `True` | Spawns the Husky and starts its supporting nodes. |
| `parrot` | Boolean | `True` | Spawns the Parrot and starts its supporting nodes. |
| `rviz` | Boolean | `True` normally; `False` with `ui:=true` | Opens one RViz window for each enabled robot, including before deployment in UI mode. |
| `slam` | Boolean | `True` | Starts SLAM Toolbox for each enabled robot. |
| `nav2` | Boolean | `True` | Starts Nav2 for each enabled robot. Nav2 currently requires `slam:=true`. |
| `husky_x` | Number (metres) | `90.0` | Husky starting x coordinate. |
| `husky_y` | Number (metres) | `-48.0` | Husky starting y coordinate. |
| `husky_z` | Number (metres) | `6.0` | Husky starting z coordinate. |
| `husky_yaw` | Number (radians) | `0.0` | Husky starting yaw angle. |
| `husky_spawn_delay` | Number (seconds) | `2.0` | Simulation-time delay before creating the Husky (wall time with `use_sim_time:=false`). |
| `parrot_x` | Number (metres) | `90.0` | Parrot starting x coordinate. |
| `parrot_y` | Number (metres) | `-48.0` | Parrot starting y coordinate. |
| `parrot_z` | Number (metres) | `8.0` | Parrot starting z coordinate. |
| `parrot_yaw` | Number (radians) | `0.0` | Parrot starting yaw angle. |
| `parrot_spawn_delay` | Number (seconds) | `2.0` | Simulation-time delay before creating the Parrot (wall time with `use_sim_time:=false`). |

> The public robot-selection arguments are named `husky` and `parrot`, not
> `husky_enabled` and `parrot_enabled`. The latter names are only variables
> inside the Python launch file.

Boolean values can be passed as `true` or `false` (case-insensitive). Multiple
arguments may be added to the same command, separated by spaces.

Because `gz_world.launch.py` includes the upstream `ros_gz_sim` launch file,
`--show-args` also lists arguments such as `gz_args`, `gz_version`, `debugger`,
and `on_exit_shutdown`. They are Gazebo implementation details rather than the
intended `rs1_sim` interface; normal launches should use the arguments in the
table above.

## Common commands

Run both robots without RViz:

```bash
ros2 launch rs1_sim rs1_sim.launch.py rviz:=false
```

Run both robots without SLAM or Nav2:

```bash
ros2 launch rs1_sim rs1_sim.launch.py slam:=false nav2:=false
```

Run only the Husky, with its RViz window:

```bash
ros2 launch rs1_sim rs1_sim.launch.py parrot:=false
```

Run only the Parrot, without RViz:

```bash
ros2 launch rs1_sim rs1_sim.launch.py husky:=false rviz:=false
```

Run only the Gazebo world:

```bash
ros2 launch rs1_sim rs1_sim.launch.py ui:=true
```

Select a different world:

```bash
ros2 launch rs1_sim rs1_sim.launch.py world:=simple_trees
```

Start both robots at custom poses:

```bash
ros2 launch rs1_sim rs1_sim.launch.py \
  husky_x:=-2.0 husky_y:=1.0 husky_yaw:=1.57 \
  parrot_x:=3.0 parrot_y:=-1.0 parrot_z:=1.5 parrot_yaw:=-1.57
```

Adjust or stagger the spawn delays:

```bash
ros2 launch rs1_sim rs1_sim.launch.py \
  husky_spawn_delay:=4.0 parrot_spawn_delay:=6.0
```

With the default `use_sim_time:=true`, creation waits on the bridged Gazebo
`/clock`. Loading the world does not consume the spawn delay, so a slow cold
start cannot trigger the create service request before the world starts
running. A paused world must be unpaused before its spawn timer can advance;
the main launch starts Gazebo running automatically. With
`use_sim_time:=false`, standalone spawners use a wall-clock delay instead, so
Gazebo must already be ready when that delay expires.

Setting `rviz:=false` disables RViz only; the Gazebo application still starts.

The `/simulation/session` topic publishes a reliable, transient-local JSON
`std_msgs/String` heartbeat at 2 Hz after the first simulation clock message.
It identifies the current world lifetime, world name, UI mode and Gazebo
partition. Heartbeats use wall time, so pausing simulation time does not stop
deployed robot nodes. Closing Gazebo ends its parent launch and heartbeat.
Each new world launch receives a new UUID; a deployment cannot be reused after
the world restarts.

## Standalone launch files

The main launch file is recommended for normal use. The smaller launch files
are also useful for testing individual parts.

Start only Gazebo and the clock bridge:

```bash
ros2 launch rs1_sim gz_world.launch.py world:=v2_forest use_sim_time:=true
```

In another terminal, spawn only the Husky into an already-running Gazebo
instance:

```bash
ros2 launch rs1_sim spawn_husky.launch.py \
  husky_x:=0.0 husky_y:=0.0 husky_z:=0.4 husky_yaw:=0.0 \
  husky_spawn_delay:=3.0 use_sim_time:=true
```

Or spawn only the Parrot:

```bash
ros2 launch rs1_sim spawn_parrot.launch.py \
  parrot_x:=2.0 parrot_y:=0.0 parrot_z:=0.8 parrot_yaw:=0.0 \
  parrot_spawn_delay:=6.0 use_sim_time:=true
```

The standalone spawn defaults differ from the main launch file:

| Launch file | Standalone spawn-delay default | Default used by `rs1_sim.launch.py` |
| --- | --- | --- |
| `spawn_husky.launch.py` | `3.0` seconds | `2.0` seconds |
| `spawn_parrot.launch.py` | `6.0` seconds | `2.0` seconds |

The main launch file passes its own delay values into the child launch files,
so its `2.0` second defaults take precedence. Standalone spawn files do not
start Gazebo or RViz, and starting one twice without removing the existing
model will cause a duplicate-model-name error. They start the robot state
publisher and bridge immediately; only model creation is delayed. Source the
workspace in every terminal before running a standalone launch file.

## Namespaces and ROS interfaces

The Husky runs in the `/husky1` namespace and the Parrot runs in the `/parrot1`
namespace. Their Gazebo entity names are `husky1` and `parrot1`, and their TF
frame names use the `husky1_` and `parrot1_` prefixes. These identities are
currently fixed rather than launch arguments, so changing a namespace alone is
not enough to spawn a second instance of either robot. Each bridge exposes the
following relative ROS topic names beneath its robot namespace:

| Topic | Direction | Purpose |
| --- | --- | --- |
| `joint_states` | Gazebo to ROS | Simulated joint state data. |
| `odometry` | Gazebo to ROS | Simulated odometry. |
| `imu` | Gazebo to ROS | Simulated IMU data. |
| `cmd_vel_safe` | ROS to Gazebo | Robot velocity command, published only by the `rs1_safety` E-stop gate. |
| `scan` | Gazebo to ROS | 2D laser scan. |
| `scan/points` | Gazebo to ROS | Lidar point cloud. |
| `camera/image` | Gazebo to ROS | RGB camera image. |
| `camera/depth/image` | Gazebo to ROS | Depth image. |
| `camera/depth/points` | Gazebo to ROS | Depth point cloud. |

For example, publish Husky velocity commands to `/husky1/cmd_vel` and Parrot
velocity commands to `/parrot1/cmd_vel`. The E-stop gate forwards
`/husky1/cmd_vel` and Parrot's terrain-following output
`/parrot1/cmd_vel_combined` to the bridges' `cmd_vel_safe` topics unless a stop
is latched. The world launch also exposes the global `/clock` topic.

Each RViz instance is namespaced and loads its robot-specific configuration:

- Husky: `config/husky1.rviz`, fixed frame `husky1_map`
- Parrot: `config/parrot1.rviz`, fixed frame `parrot1_map`

Both displays listen to global `/tf` and `/tf_static`. The alignment node
aggregates each robot's local TF and adds `world` to map transforms here, allowing
the `/mission/roi_marker` marker in frame `world` to appear in either display.
Localisation and Nav2 keep using the robot-specific TF topics. With `ui:=true`,
the world frame becomes available after deployment and map registration.

To reopen the displays against an already-running aligned stack, close the old
RViz windows and run `ros2 launch rs1_sim rviz.launch.py`. This starts only the
displays. If using `alignment:=false`, the custom stack must supply the global
TF tree, as it does for the mission nodes.

## World notes

- `large_demo` and `simple_trees` reference models from Gazebo Fuel. Their first
  launch may require network access while Gazebo downloads those assets.
- `simple_trees` references a `grass_plane` model that is not currently shipped
  by `rs1_sim`. It must be available from another installed package or the local
  Gazebo model cache.
- `v2_forest` uses `model://...` URIs for models installed by `rs1_sim`. Build
  the workspace and source its `install/setup.*` file before launching so
  Gazebo can discover those models from any workspace location.
- The `ground_station` model used by `v2_forest` includes assets from Gazebo
  Fuel, so its first launch may require network access while they are cached.

## Operational notes

- In the main launch, both model-creation actions wait for simulation time,
  with default `2.0` second delays. If no robots appear, check that `/clock`
  advances and Gazebo is unpaused. The `ros_gz_sim create` process can exit
  cleanly even after a create-service timeout; its exit status alone does not
  confirm that the model exists. Creation disallows automatic model renaming,
  so repeated requests cannot silently spawn `husky1_0` or `parrot1_0`.
- RViz starts immediately rather than waiting for model creation. Missing TF or
  sensor data during the first few seconds can therefore be transient. Use
  `rviz:=false` when conserving CPU/GPU resources or debugging Gazebo alone.
- `use_sim_time:=false` changes whether the ROS nodes use simulation time; it
  does not disable the global `/clock` bridge.
- The supplied RViz profiles include Navigation 2 panels and tools. A clean
  environment needs `nav2_rviz_plugins` to load those panels.

## Localisation and navigation

The main launch includes the robot-specific launch files from
`rs1_localisation` and `rs1_navigation`. SLAM and Nav2 default to enabled and
can be disabled independently with `slam:=false` and `nav2:=false`.

The current navigation setup uses the live map produced by SLAM Toolbox, so do
not run `nav2:=true slam:=false` until saved-map localisation with a map server
and AMCL has been added.
