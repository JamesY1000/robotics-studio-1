# sample_generator

Generates **labelled synthetic training data** for the RS1 perception stack by
spawning tree models in the Ignition Gazebo simulation, moving the Parrot drone
and capturing RGB images with matching YOLO bounding-box labels.

Output feeds the training pipeline in [`../yolo_training`](../yolo_training/README.md).

## Packages

This directory is its own colcon workspace containing:

| Package | Purpose |
| --- | --- |
| `sample_gen` | The generation nodes, launch files and config. |
| `sample_gen_msgs` | Service definitions used to drive generation. |
| `sample_gen_gazebo_plugins` | The `ObjectLabelPlugin` Gazebo system plugin that publishes ground-truth object labels. |

```bash
cd sample_generator
colcon build
. install/setup.bash
```

## The camera (tree) pipeline

The tree detector is trained on the **camera** path:

```bash
ros2 launch sample_gen generate_camera_samples.launch.py world:=simple_trees
```

That brings up the Parrot-only simulation plus:

* `model_tf_broadcaster.py` - publishes tree TFs.
* `camera_collector.py` - the node that actually builds the dataset.

`camera_collector.py` writes, under `sample_gen/database/camera_samples/`:

```
images/     <id>.jpg   720x480 rgb8 frames
labels/     <id>.txt   YOLO labels, 1-based ids (1 = OAK_TREE, 2 = PINE_TREE)
labelled/   <id>.jpg   the same frames with boxes drawn, for visual review
sample_database.csv    one row per sample, with its class
```

Class ids are **1-based** here because they come straight from the plugin's
`<classification>`; `yolo_training/prepare_dataset.py` remaps them to 0-based.

### Resume support

`camera_collector.py` resumes from the highest id in `sample_database.csv`, and
`sample_generator.py` skips batches already completed, so an interrupted run can
be restarted without duplicating samples. Batch composition is deterministic
(`batch_type = num_batch % 5`: 0/1/2 = mixed, 3 = all pine, 4 = all oak).

### Configuration

| File | Controls |
| --- | --- |
| `config/config.yml` | Per-node parameters: database dir, world name, sampling thresholds, training split. |
| `config/class_configs.yml` | Classes and `num_instances` (how many of each tree to spawn). |
| `config/spawn_areas.yml` | The polygon trees are spawned inside. |
| `config/patrol_config.yml` | Patrol waypoints. |

The `spawn_areas.yml` polygon matters a lot: trees outside the camera's view
produce no labels, so the area must sit **in front of** the drone
(`[[5,-5],[35,-20],[35,20],[5,5]]` is the tuned FOV-matching trapezoid).

## Other launch files

`generate_samples`, `generate_database`, `generate_camera_database`,
`generate_forest_samples` and `preprocess_samples` are for the original
**lidar / point-cloud** sample flow (`sample_collector.py`,
`data_preprocessor.py`) that this package was adapted from. `inference.launch.py`
contains a legacy, commented-out inference node that points at external paths
from a different project - ignore it.

## Gotchas learned the hard way

* **Rebuild after every script/config edit**: `colcon build --packages-select sample_gen`
  (this package is not symlink-installed).
* **Trees render dark grey** (~18-70), *not* green. Detect them with a
  `gray < 100` threshold, not a green-dominance test.
* **The Ignition server must be killed with `pkill -9 -f "ign gazebo"`** -
  killing only the launch PID leaves the ruby-wrapped server running and the next
  run will be confused.
* **`set_pose` returns `data: true` even when it fails** (a Fortress quirk) -
  the collector retries 3x with a 5s timeout.
* **Stale boxes** (boxes belonging to the *next* image) come from Gazebo's
  `/clock` lagging the sensor timestamps. Fixed with a 2 s sim-time freshness
  margin on both the image and the tree TFs.
* **`robot_localization` EKF drift** (~4 m) corrupted the camera pose. The drone
  is static, so `parrot_world_broadcaster.py` publishes a static
  `world -> parrot1_base_link` instead of republishing the EKF.
* `ObjectLabelPlugin` used to segfault the whole Gazebo server when it published
  before its ROS publishers existed - guarded by an `ros_ready` atomic flag.
