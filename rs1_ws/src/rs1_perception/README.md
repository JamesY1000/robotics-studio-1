# rs1_perception

Perception nodes for the RS1 project: tree detection from the Parrot drone camera
and conversion of those detections into world-frame tree locations.

## Pipeline

```
  /parrot1/camera/image
          |
          v
   [ yolo_node ]  ------------------>  /yolo/debug_image      (annotated frame)
          |  /yolo/detect (service)
          v
   YoloDetections (class_name, score, left/top/right/bottom)
          |
          v
   [ tree_location_node ]  --------->  /yolo/process_tree_locations (service)
          |  uses depth + lidar + TF to unproject each box
          v
   TreeLocation (tree_type, x, y, z in the world frame)
          |
          v
   [ scripts/map_tree_locations.py ]  ->  /tree_locations/map (MarkerArray)
```

| Node | Executable | Exposed as |
| --- | --- | --- |
| Tree detection | `tree_identification/yolo_node.py` | `/yolo/detect` service, `/yolo/debug_image` topic |
| Tree localisation | `tree_location/tree_location_processing.py` | `/yolo/process_tree_locations` service |
| World frame broadcaster | `tree_location/parrot_world_broadcaster.py` | `/tf`, `/tf_static` |

Helper clients: `scripts/test_tree_locations.py` (accuracy test),
`scripts/visualise_tree_locations.py` (RViz markers / Gazebo poles),
`scripts/map_tree_locations.py` (interactive map builder).

## Running

```bash
ros2 launch rs1_perception tree_identification.launch.py
# or pick a specific inference backend:
ros2 launch rs1_perception tree_identification.launch.py backend:=onnx
```

The launch file starts all three nodes. Everything else (image size, thresholds,
class names) lives in `config/yolo.yaml`; the launch file only exposes the
backend selection and the model paths.

## Testing the inference backend

### Quick check - no simulation needed

Proves the chosen backend loads, the service answers, and boxes come back in
original-image coordinates. Start the node in one terminal:

```bash
ros2 run rs1_perception yolo_node.py --ros-args \
    -p backend:=onnx -p confidence_threshold:=0.01
```

and in another:

```bash
ros2 run rs1_perception test_yolo_backend.py --expect-backend onnx
```

It reads the node's parameters (so it *proves* which backend is live) and prints
them plus the detections:

```
/yolo_node configuration:
  backend                onnx
  provider               auto
  imgsz                  640
  onnx_model_path        (empty -> derived from model_path)
...
returned 1 detection(s)
  PINE_TREE  0.0212  [0,0,14,47]
  all boxes are inside the 720x480 frame  OK
RESULT: PASS
```

It exits non-zero if the backend does not match `--expect-backend`, so it works
as a regression check. Note the node must run with a **low
`confidence_threshold`** for this test: the frame published is a terrain texture,
not a tree scene, so at the default `0.5` you would correctly get zero
detections.

> This verifies plumbing, **not** detection accuracy - the "detections" on a
texture are meaningless noise. Use the simulation for accuracy.

### Full pipeline - with the simulation

```bash
# terminal 1
ros2 launch rs1_sim rs1_sim.launch.py husky:=false rviz:=false slam:=false nav2:=false

# terminal 2 - the whole perception stack on the onnx backend
ros2 launch rs1_perception tree_identification.launch.py backend:=onnx

# terminal 3 - teleport around the forest and score the detections
ros2 run rs1_perception test_tree_locations.py --views 5 --seed 3
```

`test_tree_locations.py` reports position precision and mean error against the
world's ground-truth trees, which is the check that actually validates detection
quality. Swap `backend:=onnx` for `backend:=pytorch` to compare - the results
should be equivalent.

## Inference backends

`yolo_node` can run the same detector three ways. Choose with the `backend`
parameter (`config/yolo.yaml`) or the `backend:=` launch argument.

| `backend` | Runs | Needs installed | GPU |
| --- | --- | --- | --- |
| `pytorch` | `best.pt` | `ultralytics` (pulls torch + torchvision) | optional |
| `onnx` | `best.onnx` | `onnxruntime` + numpy + opencv. **No torch.** | optional |
| `tensorrt` | the **same** `best.onnx`, on ONNX Runtime's `TensorrtExecutionProvider` | `onnxruntime-gpu` + TensorRT libraries. **No torch, no ultralytics.** | **required** |
| `auto` *(default)* | first usable of `tensorrt -> onnx -> pytorch` | depends on what is found | - |

Because TensorRT executes the ONNX graph, there is only **one model file to
produce and deploy**: `best.onnx`. The hardware available at runtime decides
whether that graph executes on TensorRT, CUDA or the CPU.

`auto` logs which backend it chose and why, e.g.

```
[INFO] [yolo_node]: backend=auto selected 'onnx' -> .../models/best.onnx
       (reason: tensorrt: TensorRT engine not found: .../models/best.engine ...)
```

If a requested backend is unusable you get one clear line instead of a traceback:

```
[FATAL] [yolo_node]: TensorRT engine not found: .../models/best.engine
        (build it on the GPU machine with `python yolo_training/export_onnx.py --format engine`)
```

### Which should I use?

**`onnx`**, and it does not need PyTorch: the runtime dependency drops from
roughly **4.4 GB** to **~50 MB**, and it works on a machine with no GPU and no
CUDA at all. On an NVIDIA machine the *same* file gets TensorRT acceleration by
switching `backend` to `tensorrt` - nothing has to be re-exported or rebuilt.

The model is exported from the trained `.pt` (see
[`yolo_training/README.md`](../../../yolo_training/README.md)); the `.pt` is never
required at runtime.

### Installing the requirements

```bash
cd rs1_ws/src/rs1_perception
pip install -r requirements/onnx.txt       # recommended: torch-free
pip install -r requirements/pytorch.txt    # only if you need the pytorch backend
pip install -r requirements/tensorrt.txt   # NVIDIA only, on top of onnx.txt
```

> **`onnxruntime` and `onnxruntime-gpu` are the same import name** and cannot be
> installed side by side. For TensorRT, remove the CPU build first:
> `pip uninstall -y onnxruntime && pip install -r requirements/tensorrt.txt`.
>
> If you have no NVIDIA GPU, install the CPU-only torch wheel **before**
> `requirements/pytorch.txt`, otherwise pip pulls the CUDA build and ~3 GB of
> `nvidia/*` libraries you cannot use:
> `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`

The devcontainer already installs the ONNX profile (`.devcontainer/requirements.txt`),
so `backend:=onnx` works out of the box.

### TensorRT notes (NVIDIA users)

TensorRT is an **execution provider on the ONNX model**, not a separate artefact,
so there is no export step and no engine file to commit:

```bash
pip uninstall -y onnxruntime
pip install -r requirements/tensorrt.txt      # onnxruntime-gpu + TensorRT libs
ros2 launch rs1_perception tree_identification.launch.py backend:=tensorrt
```

Things worth knowing:

* **The engine is built on first use and takes minutes.** It is cached to
  `trt_cache_dir` (default `~/.cache/rs1_perception/tensorrt`) and reused
  afterwards. Do not mistake the first-run delay for a hang.
* **Engines are machine-specific** (GPU model + driver + CUDA + TensorRT
  version), which is exactly why they are cached locally instead of shipped.
* `provider: "tensorrt"` achieves the same thing while staying on
  `backend: "onnx"`; conversely `provider: "cpu"` is useful to keep the onnx
  backend off the GPU entirely.
* **Jetson / aarch64**: do not use the `onnxruntime-gpu` wheel - install NVIDIA's
  own onnxruntime build for your JetPack, which is compiled with the TensorRT
  provider.
* TensorRT only accelerates CUDA. On a machine without an NVIDIA GPU
  (`nvidia-smi` absent) this backend is simply unavailable - `auto` skips it and
  falls through to `onnx`.

## Configuration

`config/yolo.yaml`:

| Parameter | Default | Notes |
| --- | --- | --- |
| `backend` | `"auto"` | `auto` / `pytorch` / `onnx` / `tensorrt` |
| `provider` | `"auto"` | Execution provider for onnx/tensorrt: `auto` / `tensorrt` / `cuda` / `cpu`. |
| `trt_cache_dir` | `~/.cache/rs1_perception/tensorrt` | Where the TensorRT engine is cached. `""`/`"off"` disables caching. |
| `trt_fp16` | `true` | Allow TensorRT FP16. Ignored unless TensorRT is active. |
| `model_path` | `"best.pt"` | PyTorch weights. The launch file overrides this with an absolute path into `share/models/`. |
| `onnx_model_path` | `""` | Empty = `model_path` with the extension swapped to `.onnx`. Used by both the `onnx` and `tensorrt` backends. |
| `imgsz` | `640` | Letterbox size. pytorch accepts any value; onnx/tensorrt **must match the export size** (checked at startup). Independent of camera resolution. |
| `device` | `"auto"` | `auto` / `cpu` / `0`. Only used by the pytorch backend. |
| `confidence_threshold` | `0.5` | All backends. |
| `iou_threshold` | `0.45` | pytorch always; onnx/tensorrt for raw-head exports. |
| `class_names` | `[""]` | Override class names; empty = read them from the model. |
| `publish_debug_image` | `true` | Publish the annotated frame. |
| `image_topic` / `debug_image_topic` | `/parrot1/camera/image` / `/yolo/debug_image` | |

Class names are read from the model itself: the ONNX metadata for the onnx and
tensorrt backends (Ultralytics embeds `names`), `model.names` for pytorch.

**Model path resolution.** An absolute `model_path` / `onnx_model_path` is used
as-is. A *relative* one (such as the `best.pt` default) is looked for in the
current working directory first and then in the package's installed
`share/rs1_perception/models/` directory. That means
`ros2 run rs1_perception yolo_node.py --ros-args -p backend:=onnx` works from any
directory without having to spell out the path - the launch file just passes the
absolute path explicitly for clarity.

### Threshold behaviour per backend

* **pytorch** - `confidence_threshold` and `iou_threshold` both apply.
* **onnx / tensorrt with a raw-head export** *(default export)* - both apply,
  because the node decodes and NMS-filters the graph output itself. Behaves like
  pytorch.
* **onnx / tensorrt with an end-to-end export** (`export_onnx.py --e2e`) - NMS and
  a confidence floor are baked into the graph. `iou_threshold` has no effect, and
  `confidence_threshold` cannot go below the `--conf` used at export time.

## Measured performance

Measured on the dev machine (AMD Ryzen 5 7535U, 12 threads, **no GPU**), same
image and 30 iterations:

| Backend | mean | ~FPS |
| --- | --- | --- |
| `pytorch` | 40-49 ms | 21-25 |
| `onnx` (raw head) | ~41 ms | ~24 |
| `onnx` via ultralytics | 69 ms | 14 |
| `tensorrt` | GPU only - not measurable here | - |
Two conclusions worth keeping in mind:

1. **Swapping runtimes buys essentially nothing on CPU.** ONNX and PyTorch are
   within noise of each other; going through Ultralytics' own ONNX path is
   actively *slower* because of its wrapper overhead. Choose `onnx` for the
   dependency/portability win, not for speed.
2. **The effective lever is `imgsz`**, not the runtime: 640 -> 480 was ~1.8x
   faster. The model was trained at 640, so re-validate accuracy with
   `yolo_training` before changing it.

Inference is also not the bottleneck of this pipeline - it takes ~40 ms versus
a multi-second simulation snapshot cycle.

## Gotchas

* **`install(PROGRAMS)` executes the `.py` files directly**, so every node must
  keep its `#!/usr/bin/env python3` shebang and its executable bit. A missing
  shebang shows up as `OSError: [Errno 8] Exec format error`.
  Check with `head -c 16 <file> | xxd` (expect `2321 2f75 7372...`).
* **Never import `cv_bridge`.** ROS 2 Humble's `cv_bridge` is built against
  NumPy 1.x while the system Python has NumPy 2.x, which leads to
  `AttributeError: _ARRAY_API not found` and then a SIGSEGV. Images are decoded
  manually with `np.frombuffer(bytes(msg.data))` instead.
* **`imgsz` must match the ONNX export.** The exported graphs have a *static*
  input size, so the `onnx` and `tensorrt` backends only accept the size they were
  exported at. Changing `imgsz` is therefore fine on `pytorch` (Ultralytics
  re-letterboxes per call) but needs a re-export for the others:
  `export_onnx.py --format onnx --imgsz 480`. A mismatch is reported once at
  startup, and `backend: auto` falls through to a backend that does accept it,
  rather than failing on every detection request.
* **`imgsz` is unrelated to camera resolution.** 720x480 frames are letterboxed to
  `imgsz` automatically; changing the camera resolution does not require changing
  `imgsz`.
* **Exported models are large binaries.** `models/best.onnx` is ~9.8 MB; decide
  deliberately whether to commit it (committing it makes the torch-free
  deployment story work out of the box) or to generate it during deployment.
* **`onnxruntime` and `onnxruntime-gpu` collide** - they are two builds of the
  same `onnxruntime` import. Install only one; uninstall the CPU build before
  installing the GPU one.
* **The first TensorRT run is slow** (minutes) because the engine is being built;
  it is then cached in `trt_cache_dir`. Delete that directory to force a rebuild
  after upgrading TensorRT or the GPU driver.
* Scripts started from a launch file receive `--ros-args ...` appended, so plain
  `argparse` rejects them. Use `remove_ros_args(sys.argv)[1:]`.
* **Stale `install/` breaks things silently.** If the pipeline stops working
  after a branch switch, rebuild before debugging:
  `colcon build` (or at least `rs1_interfaces` + `rs1_perception`).
