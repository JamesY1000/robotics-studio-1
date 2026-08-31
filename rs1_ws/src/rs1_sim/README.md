# `rs1_sim`

`rs1_sim` brings up the Gazebo simulation environment for Robotics Studio 1. It
can load one of the included world SDFs, spawn the Husky and Parrot robots, bridge
their Gazebo interfaces to ROS 2, publish their robot descriptions and TF, and
open a separate RViz window for each enabled robot, and include each robot's
localisation, SLAM, and Nav2 launch files.

## Package responsibilities

| Launch file | Responsibility |
| --- | --- |
| `rs1_sim.launch.py` | Main entry point. Starts the world, selects which robots to spawn, and optionally starts RViz, SLAM, and Nav2. |
| `gz_world.launch.py` | Starts Gazebo with a selected world and bridges the Gazebo `/clock` topic to ROS 2. |
| `spawn_husky.launch.py` | Publishes the Husky description and TF, starts its Gazebo bridge, and creates the `husky1` model. |
| `spawn_parrot.launch.py` | Publishes the Parrot description and TF, starts its Gazebo bridge, and creates the `parrot1` model. |

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
and both RViz windows:

```bash
ros2 launch rs1_sim rs1_sim.launch.py
```

Launch arguments use the ROS 2 syntax `argument_name:=value`. To see the
arguments exposed by the installed launch file:

```bash
ros2 launch rs1_sim rs1_sim.launch.py --show-args
```

## Main launch arguments

These are the public arguments for `rs1_sim.launch.py`.

| Argument | Type / allowed values | Default | Purpose |
| --- | --- | --- | --- |
| `world` | `large_demo`, `simple_trees`, `v1_forest`, or `v2_forest` | `v2_forest` | Gazebo world to load. The `.sdf` extension is added automatically. |
| `use_sim_time` | Boolean | `True` | Makes ROS nodes use the Gazebo `/clock` simulation time. |
| `husky` | Boolean | `True` | Spawns the Husky and starts its supporting nodes. |
| `parrot` | Boolean | `True` | Spawns the Parrot and starts its supporting nodes. |
| `rviz` | Boolean | `True` | Opens one RViz window for each enabled robot. |
| `slam` | Boolean | `True` | Starts SLAM Toolbox for each enabled robot. |
| `nav2` | Boolean | `True` | Starts Nav2 for each enabled robot. Nav2 currently requires `slam:=true`. |
| `husky_x` | Number (metres) | `0.0` | Husky starting x coordinate. |
| `husky_y` | Number (metres) | `0.0` | Husky starting y coordinate. |
| `husky_z` | Number (metres) | `0.4` | Husky starting z coordinate. |
| `husky_yaw` | Number (radians) | `0.0` | Husky starting yaw angle. |
| `husky_spawn_delay` | Number (seconds) | `2.0` | Wall-clock delay before creating the Husky in Gazebo. |
| `parrot_x` | Number (metres) | `2.0` | Parrot starting x coordinate. |
| `parrot_y` | Number (metres) | `0.0` | Parrot starting y coordinate. |
| `parrot_z` | Number (metres) | `0.8` | Parrot starting z coordinate. |
| `parrot_yaw` | Number (radians) | `0.0` | Parrot starting yaw angle. |
| `parrot_spawn_delay` | Number (seconds) | `2.0` | Wall-clock delay before creating the Parrot in Gazebo. |

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
ros2 launch rs1_sim rs1_sim.launch.py husky:=false parrot:=false rviz:=false
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

Change the spawn delays if Gazebo needs more startup time:

```bash
ros2 launch rs1_sim rs1_sim.launch.py \
  husky_spawn_delay:=4.0 parrot_spawn_delay:=6.0
```

The delay is a startup buffer, not a readiness check. A delay of `0.0` is
allowed, but spawning can fail if Gazebo has not finished starting.

Setting `rviz:=false` disables RViz only; the Gazebo application still starts.

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
| `cmd_vel` | ROS to Gazebo | Robot velocity command. |
| `scan` | Gazebo to ROS | 2D laser scan. |
| `scan/points` | Gazebo to ROS | Lidar point cloud. |
| `camera/image` | Gazebo to ROS | RGB camera image. |
| `camera/depth/image` | Gazebo to ROS | Depth image. |
| `camera/depth/points` | Gazebo to ROS | Depth point cloud. |

For example, publish Husky velocity commands to `/husky1/cmd_vel` and Parrot
velocity commands to `/parrot1/cmd_vel`. The world launch also exposes the
global `/clock` topic.

Each RViz instance is namespaced and loads its robot-specific configuration:

- Husky: `config/husky1.rviz`, fixed frame `husky1_base_link`
- Parrot: `config/parrot1.rviz`, fixed frame `parrot1_base_link`

## World notes

- `large_demo` and `simple_trees` reference models from Gazebo Fuel. Their first
  launch may require network access while Gazebo downloads those assets.
- `simple_trees` references a `grass_plane` model that is not currently shipped
  by `rs1_sim`. It must be available from another installed package or the local
  Gazebo model cache.
- `v2_forest` currently contains absolute asset paths for the project
  devcontainer (`/robotics-studio-1/rs1_ws/install/...`). It works in that
  layout, but those paths must be made portable before the package can run from
  an arbitrary workspace location.

## Operational notes

- In the main launch, both model-creation actions run concurrently after their
  default `2.0` second delays. If Gazebo starts slowly, increase or stagger
  `husky_spawn_delay` and `parrot_spawn_delay`.
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
