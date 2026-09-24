# YOLO sample generation for the rs1_ws trees (Parrot RGB camera)

This documents how to make the **sample generator** (`sample_generator/`) produce
**YOLO-format image samples** of the trees in `rs1_ws/src/rs1_sim/models/trees`
using the **Parrot drone's RGB camera** (instead of the original lidar pipeline).

## 1. What was changed and why

### 1.1 Sim compatibility (the big one)
The original sample generator stack was written for **Gazebo Harmonic / gz-sim8**
(the `gz` CLI, `gz::sim` C++ API, `ros_gz`). This machine only has
**Ignition Gazebo Fortress (ign-gazebo6)** — the same generation the `rs1_ws`
provided simulation uses. Everything was therefore ported:

| Component | Before (Harmonic) | After (Fortress) |
|---|---|---|
| `object_label_plugin` (C++) | `gz::sim` / `GZ_ADD_PLUGIN` / `gz-sim8` | `ignition::gazebo` / `IGNITION_ADD_PLUGIN` / `ignition-gazebo6` |
| spawn/delete/set-pose (`helpers.py`) | `gz service` / `gz.msgs.*` | `ign service` / `ignition.msgs.*` |
| ROS <-> Gazebo | `ros_gz` | `ros_ign` (already used by rs1_ws) |

### 1.2 Ground-truth plugin placement (important)
The `ObjectLabelPlugin` must be attached to the spawned trees for ground truth.
It is **not** put in the shared `rs1_ws/.../trees/*/model.sdf` (those are also
used as world decoration in `simple_trees.sdf` — they would otherwise pollute
the ground truth). Instead the plugin lives in **labeled copies**:

- `sample_generator/sample_gen/models/trees/oak_tree_labeled/` (class 1)
- `sample_generator/sample_gen/models/trees/pine_tree_labeled/` (class 2)

These reuse the original meshes via `model://trees/...` (resolved with
`IGN_GAZEBO_RESOURCE_PATH`).

### 1.3 New camera/YOLO nodes
- `scripts/camera_collector.py` – subscribes to the Parrot RGB image + all
  `ObjectLabel` topics, projects each 3D tree box into the image and saves
  `images/<id>.jpg` + `labels/<id>.txt` in YOLO format. Exposes the same
  services as the old `sample_collector.py`, so `model_importer.py` and
  `sample_generator.py` drive it unchanged.
- `scripts/parrot_world_broadcaster.py` – republishes `/parrot1/tf(+static)` onto
  the global `/tf` and adds a static `world -> parrot1_odom` (the drone's spawn
  pose) so `world -> camera` and the plugin's `world -> tree` transforms share
  the same frame.
- `launch/generate_camera_database.launch.py` – import step (build `model_db.csv`)
- `launch/generate_camera_samples.launch.py` – sample-collection step

## 2. Prerequisites / build

```bash
# 1) Python runtime deps (not installed on the stripped system Python; use your
#    devcontainer or `sudo apt install python3-pip` first):
#    python3 -m pip install -r sample_generator/requirements.txt
#    (numpy<2 is fine here; the machine has numpy 1.21)

# 2) Build the ROS packages
source /opt/ros/humble/setup.bash
cd sample_generator
colcon build --packages-select sample_gen_msgs sample_gen_gazebo_plugins sample_gen
source install/setup.bash

# 3) Build the rs1_ws packages (for 41068_ignition_bringup)
cd ../rs1_ws
colcon build            # or --packages-select 41068_ignition_bringup
source install/setup.bash
```

## 3. Workflow

### Step A – import the trees (build `model_db.csv`)
```bash
ros2 launch sample_gen generate_camera_database.launch.py
```
- Launches the Parrot-only bringup in `simple_trees`, `camera_collector`,
  `parrot_world_broadcaster`, `model_tf_broadcaster`, then `model_importer`.
- `model_importer` spawns `oak_tree_labeled` then `pine_tree_labeled`, reads
  their bounding boxes from the collector, and writes
  `<database_dir>/model_db.csv`.
- Shuts down when done.

### Step B – collect YOLO samples
```bash
ros2 launch sample_gen generate_camera_samples.launch.py
```
- Spawns batches of trees inside the spawn area, moves them around, and the
  camera collector saves each RGB frame + YOLO labels to
  `<database_dir>/camera_samples/{images,labels,labelled}/` plus
  `sample_database.csv`.
- The `labelled/` folder contains previews with boxes drawn – **check these
  first** to verify the projection aligns with the trees.

## 4. Configuration (config/config.yml)
- `model_importer_node.model_names` / `model_dir` – the labeled tree models.
- `camera_collector_node` – `image_topic`, `camera_frame`
  (`parrot1_camera_rgb_optical_frame`), `hfov`, `database_dir`.
- `sample_generator_node` – `world_name: simple_trees`, spawn area, batch counts.
- `parrot_world_broadcaster_node.spawn_pose` – must match the drone spawn in
  `41068_ignition.launch.py` (default `[2, 0, 0.8, 0]`).
- `class_configs.yml` – `OAK_TREE` id 1, `PINE_TREE` id 2 (`num_instances` per
  batch). The plugin `<classification>` in the labeled models must match these.

## 5. Notes & caveats
- **Camera projection convention**: the collector projects in the Parrot's
  **optical frame** (`camera_rgb_optical_frame`) with a standard pinhole model
  (`u = fx*X/Z + cx`, `+Z` forward). Intrinsics are computed from `hfov` +
  image size (no `camera_info` is bridged). If the preview boxes are mirrored or
  misplaced, double-check the camera frame / convention.
- **Drone viewpoint**: the Parrot hovers at the spawn pose (2, 0, 0.8 m, camera
  pitched 45° down). Trees are placed in a 20x20 m area around it, so the camera
  sees the nearest ones. Raise the drone (edit the spawn in
  `41068_ignition.launch.py` + `spawn_pose`) for more top-down coverage.
- **Simple_trees world**: the two decorative fuel trees were switched to local
  `model://trees/...` references (no network needed).
- **Occlusion**: trees placed `distance: 2.0` m apart; some mutual occlusion is
  possible and acceptable for YOLO training.
- **Static trees**: `set_pose` is used to move them; this works for `<static>`
  models in ign-gazebo.
