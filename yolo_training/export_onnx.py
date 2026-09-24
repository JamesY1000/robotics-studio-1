#!/usr/bin/env python3
"""Export the tree detector to ONNX / TensorRT for deployment.
Typical use - build the ONNX model used by the ``onnx`` inference backend::

    python export_onnx.py --format onnx

Then run the node on it (no PyTorch needed at runtime)::

    ros2 launch rs1_perception tree_identification.launch.py backend:=onnx

Two ONNX flavours are available.  The default is the **raw head**
``(1, 4+nc, M)``: ``xywh`` boxes plus per-class scores, which the node decodes
and NMS-filters itself, so ``confidence_threshold`` and ``iou_threshold`` are
fully honoured at runtime and behave exactly like the pytorch backend::

    python export_onnx.py --format onnx

With ``--e2e`` you instead get the self-contained **end-to-end** graph
(``nms=True``) whose output is the ready-to-use ``(1, N, 6)`` tensor
``x1, y1, x2, y2, confidence, class_id``.  That is simpler to consume from a
non-ROS tool, but NMS and a confidence floor (``--conf``, default 0.05) are
frozen into the graph at export time::

    python export_onnx.py --format onnx --e2e

On an NVIDIA machine, TensorRT acceleration needs **no extra export step**: the
``tensorrt`` inference backend runs the same ``.onnx`` through ONNX Runtime's
``TensorrtExecutionProvider``, so the engine is built and cached automatically on
first use::

    python export_onnx.py --format onnx
    ros2 launch rs1_perception tree_identification.launch.py backend:=tensorrt

.. warning::
   A TensorRT engine is built and cached on the machine that runs it (see
   ``trt_cache_dir``), because it is specific to the GPU model, driver, CUDA and
   TensorRT versions in use.  It is therefore not something you ship - you ship
   the ``.onnx`` and let each machine build its own engine once.

The default output directory is ``rs1_ws/src/rs1_perception/models`` so the ROS
package can pick the artefacts up straight away.  Remember to rebuild the
workspace (``colcon build --packages-select rs1_perception``) so the new files are
copied into ``install/.../share/rs1_perception/models/``.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = (
    REPO_ROOT / "rs1_ws" / "src" / "rs1_perception" / "models" / "best.pt"
)
DEFAULT_OUT_DIR = DEFAULT_WEIGHTS.parent

# Ultralytics writes the artefact next to the .pt; these are the extensions we
# know how to relocate to --out-dir.
#
# NOTE: there is deliberately no TensorRT target here.  The `tensorrt` inference
# backend executes the *same* .onnx through ONNX Runtime's
# TensorrtExecutionProvider, so there is no separate engine file to build or
# deploy - build the onnx and the runtime picks the provider.
_FORMATS = ("onnx", "openvino")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Export the tree detector to ONNX / TensorRT / OpenVINO.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--weights",
        default=str(DEFAULT_WEIGHTS),
        help="Trained PyTorch weights (.pt) to export.",
    )
    p.add_argument(
        "--format",
        dest="formats",
        default="onnx",
        help=(
            "Comma-separated export targets: 'onnx', 'openvino', or 'all'. "
            "TensorRT is not an export target: run the onnx artefact on it via "
            "the node's backend:=tensorrt."
        ),
    )
    p.add_argument(
        "--out-dir",
        default=str(DEFAULT_OUT_DIR),
        help="Directory to copy the exported artefacts into.",
    )
    p.add_argument("--imgsz", type=int, default=640, help="Model input size (px).")
    p.add_argument("--opset", type=int, default=17, help="ONNX opset version.")
    p.add_argument(
        "--conf",
        type=float,
        default=0.05,
        help=(
            "Confidence floor baked into an end-to-end ONNX graph. The node's "
            "confidence_threshold cannot go below this, so keep it low; use "
            "--raw-head if you need it fully free."
        ),
    )
    p.add_argument(
        "--iou",
        type=float,
        default=0.45,
        help="IoU threshold baked into an end-to-end ONNX graph.",
    )
    p.add_argument(
        "--max-det",
        type=int,
        default=300,
        help="Maximum detections kept in an end-to-end ONNX graph.",
    )
    p.add_argument(
        "--e2e",
        action="store_true",
        help=(
            "Export the self-contained end-to-end graph (NMS baked in) instead of "
            "the raw head. Simpler output, but NMS and the confidence floor are "
            "frozen at export time, so the node's thresholds cannot be tuned."
        ),
    )
    p.add_argument(
        "--device",
        default="cpu",
        help="Export device: 'cpu' or a GPU index such as '0' (OpenVINO/GPU targets).",
    )
    p.add_argument(
        "--half",
        action="store_true",
        help="FP16 export (OpenVINO/GPU targets only).",
    )
    p.add_argument(
        "--int8",
        action="store_true",
        help="INT8 export (OpenVINO; needs a calibration dataset for best results).",
    )
    p.add_argument(
        "--no-simplify",
        dest="simplify",
        action="store_false",
        help="Skip onnxslim graph simplification for ONNX.",
    )
    p.add_argument(
        "--verify",
        action="store_true",
        help="Load the exported ONNX with onnxruntime and check its output shape.",
    )
    return p.parse_args()


def resolve_formats(spec: str) -> list[str]:
    wanted = [f.strip().lower() for f in spec.split(",") if f.strip()]
    if "all" in wanted:
        return list(_FORMATS)
    unknown = [f for f in wanted if f not in _FORMATS]
    if unknown:
        raise SystemExit(
            f"unknown export format(s): {', '.join(unknown)} "
            f"(expected one of: {', '.join(_FORMATS)}, all)"
        )
    return wanted


def _export_one(model, fmt: str, args) -> Path | None:
    """Run the Ultralytics export for a single format and return the artefact."""
    kwargs = {
        "format": fmt,
        "imgsz": args.imgsz,
        "device": args.device,
    }

    if fmt == "onnx":
        kwargs.update(opset=args.opset, simplify=args.simplify, dynamic=False)
        if not args.e2e:
            # Raw head: (1, 4+nc, M) xywh boxes + per-class scores.  The node
            # decodes and NMS-filters this itself, so confidence_threshold and
            # iou_threshold stay fully controllable at runtime, matching the
            # pytorch backend.
            #
            # NOTE: the `nms` argument must be OMITTED entirely to get the raw
            # head - passing nms=False still yields the end-to-end graph.
            pass
        else:
            # Self-contained graph: NMS baked in, output is the ready-to-use
            # (1, N, 6) = x1, y1, x2, y2, confidence, class_id tensor.  `conf`
            # becomes a *floor* in the graph, so keep it low.
            kwargs.update(nms=True, conf=args.conf, iou=args.iou, max_det=args.max_det)
    elif fmt == "openvino":
        kwargs.update(dynamic=False, half=args.half, int8=args.int8)

    try:
        produced = model.export(**kwargs)
    except Exception as ex:  # noqa: BLE001 - surface Ultralytics' reason verbatim
        print(f"  !! {fmt} export FAILED: {type(ex).__name__}: {ex}")
        return None

    return Path(produced) if produced else None


def _relocate(artefact: Path, out_dir: Path) -> Path:
    """Copy the exported artefact into ``out_dir`` (unless it is already there)."""
    target = out_dir / artefact.name
    if artefact.resolve() == target.resolve():
        return target
    if artefact.is_dir():
        if target.exists():
            shutil.rmtree(target)
        shutil.copytree(artefact, target)
    else:
        shutil.copy2(artefact, target)
    return target


def _verify_onnx(path: Path, imgsz: int) -> bool:
    """Check the ONNX loads under onnxruntime and has the expected output shape."""
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as ex:
        print(f"  -- verify skipped: {ex}")
        return False

    try:
        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        providers = [
            p
            for p in ("CUDAExecutionProvider", "CPUExecutionProvider")
            if p in ort.get_available_providers()
        ]
        session = ort.InferenceSession(str(path), options, providers=providers)
        inputs = session.get_inputs()[0]
        outputs = session.get_outputs()[0]

        print(f"  input  {inputs.name} {tuple(inputs.shape)}  ({session.get_providers()[0]})")
        print(f"  output {outputs.name} {tuple(outputs.shape)}")

        shape = tuple(outputs.shape)
        if len(shape) == 3 and shape[-1] == 6:
            print(
                "  OK: self-contained end-to-end (1, N, 6) graph. NMS is baked in - "
                "the node's confidence_threshold must be >= the --conf used here."
            )
        elif len(shape) == 3:
            print(
                "  OK: raw-head graph. The node decodes it and applies NMS, so "
                "confidence_threshold and iou_threshold both apply at runtime."
            )
        else:
            print(
                "  WARNING: expected a 3-D (1, N, 6) or (1, 4+nc, M) output; the "
                "yolo_node onnx backend will refuse this graph."
            )

        # Smoke-test a forward pass on a blank frame.
        blank = np.zeros((1, 3, imgsz, imgsz), dtype=np.float32)
        session.run(None, {inputs.name: blank})
        print("  OK: forward pass succeeded.")
        return True
    except Exception as ex:  # noqa: BLE001
        print(f"  !! verify FAILED: {type(ex).__name__}: {ex}")
        return False


def main() -> int:
    args = parse_args()
    formats = resolve_formats(args.formats)

    weights = Path(args.weights).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()

    if not weights.is_file():
        raise SystemExit(
            f"weights not found: {weights}\n"
            "Train a model first (yolo_training/train_yolo.py) or pass --weights."
        )
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            "export needs the training environment:\n"
            "  cd yolo_training && python -m venv .venv && . .venv/bin/activate\n"
            "  pip install -r requirements.txt"
        ) from None

    print(f"weights : {weights}  ({weights.stat().st_size / 1e6:.2f} MB)")
    print(f"targets : {', '.join(formats)}   imgsz={args.imgsz}")
    print(f"out-dir : {out_dir}")
    print()

    model = YOLO(str(weights))

    exported: dict[str, Path] = {}
    for fmt in formats:
        print(f"==> exporting {fmt}")
        produced = _export_one(model, fmt, args)
        if produced is None:
            continue
        relocatable = produced.is_file() or produced.is_dir()
        if not relocatable or not produced.exists():
            print(f"  !! expected artefact not found at {produced}")
            continue
        final = _relocate(produced, out_dir)
        size = (
            sum(f.stat().st_size for f in final.rglob("*") if f.is_file())
            if final.is_dir()
            else final.stat().st_size
        )
        print(f"  -> {final}  ({size / 1e6:.2f} MB)")
        exported[fmt] = final

        if fmt == "onnx" and args.verify:
            _verify_onnx(final, args.imgsz)
        print()

    if not exported:
        print("nothing was exported.")
        return 1

    print("done. exported:")
    for fmt, path in exported.items():
        print(f"  {fmt:<9} {path}")

    print(
        "\nnext steps:\n"
        "  1. colcon build --packages-select rs1_perception   (to copy into install/)\n"
        "  2. ros2 launch rs1_perception tree_identification.launch.py backend:=auto\n"
        "     ('auto' prefers tensorrt -> onnx -> pytorch and logs its choice)\n"
        "     or force one with backend:=onnx / backend:=pytorch / backend:=tensorrt"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
