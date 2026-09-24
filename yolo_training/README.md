# yolo_training

Training, evaluation and deployment-export for the RS1 tree detector
(`OAK_TREE` / `PINE_TREE`).

Everything here runs in its **own virtualenv** - this is the only part of the repo
that needs a PyTorch/training stack. The ROS package that *runs* the model
(`rs1_ws/src/rs1_perception`) does not need any of it, and its default ONNX
backend needs no PyTorch at all (see `rs1_ws/src/rs1_perception/README.md`).

## Layout

| Path | Purpose |
| --- | --- |
| `consolidate_data.py` | Merge raw sample sets from `to_check/` into one uniquely-numbered `database/`. |
| `prepare_dataset.py` | Build the clean Ultralytics dataset under `datasets/trees/` (train/val split, 1-based -> 0-based class ids). |
| `train_yolo.py` | Train / validate the detector. |
| `export_onnx.py` | Export the trained `.pt` to ONNX / TensorRT / OpenVINO for deployment. |
| `database/` | Consolidated images + labels + `sample_database.csv` + `excluded.csv`. |
| `datasets/trees/` | Generated Ultralytics dataset (`images/`, `labels/`, `data.yaml`). Wiped and rebuilt by `prepare_dataset.py`. |
| `runs/` | Training output (`runs/trees/weights/best.pt`). |
| `to_check/` | Raw sample sets awaiting manual review before consolidation. |
| `.venv/` | Training virtualenv (not committed). |

## Setup

```bash
cd yolo_training
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` is pinned to `ultralytics==8.4.135`, the version that produced
the deployed model (`rs1_perception/models/best.pt`), so re-exporting reproduces
the same graph. It also pulls `onnx`/`onnxslim` (export) and `onnxruntime`
(verification).

## Workflow

```bash
# 1. Generate labelled images from the simulation (separate package)
#    see ../sample_generator/README.md

# 2. Consolidate + prepare the dataset
.venv/bin/python consolidate_data.py
.venv/bin/python prepare_dataset.py

# 3. Train
.venv/bin/python train_yolo.py --epochs 100 --imgsz 640

# 4. Deploy: export the artefacts the perception package consumes
.venv/bin/python export_onnx.py --format onnx
```

Step 3 writes `runs/trees/weights/best.pt`. **Copy that to the ROS package** so
the node can find it:

```bash
cp runs/trees/weights/best.pt ../rs1_ws/src/rs1_perception/models/best.pt
```

Then re-run step 4 (the export reads from `rs1_perception/models/best.pt`) and
rebuild the workspace so the artefacts are installed:

```bash
cd ../rs1_ws && colcon build --packages-select rs1_perception
```

## Class ids

The sample generator labels with the **1-based** ids from the Gazebo
`ObjectLabelPlugin` (`1 = OAK_TREE`, `2 = PINE_TREE`). Ultralytics expects
**0-based** ids, so `prepare_dataset.py` rewrites the labels (`1 -> 0`, `2 -> 1`)
and the deployed model's `names` are `{0: 'OAK_TREE', 1: 'PINE_TREE'}`. The raw
`database/` directory is never modified.

## Exporting (`export_onnx.py`)

Training always happens in PyTorch - ONNX/TensorRT/OpenVINO are *export targets*
produced from the `.pt`. **Exporting never retrains anything and does not change
the detections**: the exported graph is a trace of the same weights, and was
measured to agree with the PyTorch output to ~6e-4 on the raw logits (float32
rounding).

```bash
# ONNX, raw-head flavour (default) - recommended
.venv/bin/python export_onnx.py --format onnx

# ONNX, self-contained end-to-end graph (NMS baked in)
.venv/bin/python export_onnx.py --format onnx --e2e

# Alternative CPU runtime (benchmarks about the same as PyTorch on an AMD CPU)
.venv/bin/python export_onnx.py --format openvino

# Everything
.venv/bin/python export_onnx.py --format all --device 0
```

**There is no TensorRT export target.** The `tensorrt` inference backend runs the
*same* `.onnx` through ONNX Runtime's `TensorrtExecutionProvider`, so the engine
is built and cached automatically by the runtime (on the GPU machine) instead of
being produced here. One artefact, three execution providers:

```bash
ros2 launch rs1_perception tree_identification.launch.py backend:=onnx       # CPU/CUDA
ros2 launch rs1_perception tree_identification.launch.py backend:=tensorrt   # NVIDIA
```

| Option | Default | Notes |
| --- | --- | --- |
| `--format` | `onnx` | `onnx`, `openvino`, or `all` (comma separated). |
| `--weights` | `rs1_perception/models/best.pt` | Input `.pt`. |
| `--out-dir` | `rs1_perception/models/` | Where the artefact is copied. |
| `--imgsz` | `640` | **Must match** the `imgsz` the node uses. |
| `--conf` / `--iou` / `--max-det` | `0.05` / `0.45` / `300` | Only used by `--e2e`, where they are baked into the graph. |
| `--half` / `--int8` | off | OpenVINO/GPU targets. |
| `--e2e` | off | Bake NMS into the graph instead of exporting the raw head. |
| `--verify` | off | Load the result with `onnxruntime` and check the output shape + a forward pass. |

### Which ONNX flavour?

The node understands both and picks based on the graph's output shape:

* **raw head** (default) - output `(1, 4+nc, M)`. The node decodes and
  NMS-filters it, so `confidence_threshold` **and** `iou_threshold` are honoured
  at runtime, exactly like the PyTorch backend. **Use this one.**
* **end-to-end** (`--e2e`) - output `(1, N, 6)`, ready to use without any NMS
  code. Simpler for a non-ROS consumer, but NMS and the confidence floor are
  frozen at export time, so the node's thresholds cannot go below them.

> Gotcha: Ultralytics only emits the raw head when the `nms` argument is **left
> unset**. Passing `nms=False` still produces the end-to-end graph - which is why
> the flag here is `--e2e` (opt in) rather than `--no-nms`.

## Known issues

* **`prepare_dataset.py` has unresolved merge-conflict markers** (from branch
  `feat/yolo`) and currently fails to compile. It must be resolved before the
  dataset pipeline will run - the `HEAD` side (which keeps the
  `database/excluded.csv` support) is the newer variant.

## Notes / gotchas

* `prepare_dataset.py` **wipes** `datasets/trees/` (including stale `*.cache`) on
  every run - treat that directory as disposable.
* The training venv pins `ultralytics==8.4.135` while the system Python used by
  the ROS node may have a newer one. Prefer the venv for anything that writes
  model artefacts.
* Exports are written next to the input `.pt` first and then copied to
  `--out-dir`, so a stray `best.onnx` can appear in `models/` if an export is
  interrupted - `export_onnx.py` cleans this up by relocating it.
