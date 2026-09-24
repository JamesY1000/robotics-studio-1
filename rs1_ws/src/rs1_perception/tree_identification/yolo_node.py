#!/usr/bin/env python3
"""YOLO tree identification for the RS1 Parrot camera.

Runs tree detection on the latest RGB frame and returns the detections through
the ``/yolo/detect`` service.

There are three interchangeable inference backends, selected with the
``backend`` parameter (see ``config/yolo.yaml``):

``pytorch``
    Ultralytics + PyTorch on ``best.pt``.  Requires ``ultralytics``, which pulls
    in torch + torchvision (several GB, plus ~3 GB more of CUDA libraries if you
    install the CUDA wheel on a machine without an NVIDIA GPU).
``onnx``
    ONNX Runtime on ``best.onnx``.  Requires only ``onnxruntime`` (~50 MB) plus
    numpy/opencv.  **torch is never imported on this path.**
``tensorrt``
    The *same* ``best.onnx``, executed by ONNX Runtime's
    ``TensorrtExecutionProvider``.  Requires an NVIDIA GPU plus the GPU build of
    onnxruntime and the TensorRT libraries - but **not** ultralytics and **not**
    torch.  The TensorRT engine is built from the ONNX graph on first use and
    cached on disk (see ``trt_cache_dir``).

``backend: auto`` (the default) picks the first usable backend in the order
tensorrt -> onnx -> pytorch and logs which one it chose and why.

Because TensorRT runs the ONNX artefact, there is only ever one model file to
produce and deploy: ``python yolo_training/export_onnx.py --format onnx``.  The
hardware available at runtime (``provider``) decides whether it executes on
TensorRT, CUDA or the CPU.

Backend notes:
    * ``imgsz`` controls letterboxing.  The pytorch backend accepts any value
      (Ultralytics re-letterboxes per call), but the onnx/tensorrt graphs have a
      STATIC input size, so there ``imgsz`` must equal the export size - a
      mismatch is reported at startup instead of failing every inference.  It is
      independent of the camera resolution.
    * ``confidence_threshold`` is honoured by every backend.  An end-to-end ONNX
      export (``export_onnx.py --e2e``) bakes a confidence floor into the graph,
      so the threshold cannot go below it on that flavour.
    * ``iou_threshold`` is honoured by the pytorch backend and by the onnx/
      tensorrt backends when they load a raw-head export (the default), which
      decodes and NMS-filters the graph output itself.
"""

from __future__ import annotations

import ast
import os
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
from rs1_interfaces.msg import BoundingBox, YoloDetections
from rs1_interfaces.srv import DetectYolo

# NOTE: nothing heavy is imported at module scope on purpose.  ``ultralytics``
# (and therefore torch) is imported lazily inside UltralyticsBackend so that the
# onnx backend can run on a machine without PyTorch installed.
try:
    import cv2
except ImportError:  # pragma: no cover - only the onnx backend needs opencv
    cv2 = None

# Backend identifiers.
BACKEND_AUTO = "auto"
BACKEND_PYTORCH = "pytorch"
BACKEND_ONNX = "onnx"
BACKEND_TENSORRT = "tensorrt"

# Order used by ``backend: auto``: prefer the most optimised usable artefact.
_AUTO_ORDER = (BACKEND_TENSORRT, BACKEND_ONNX, BACKEND_PYTORCH)

# Box colours per class id, used when we have to draw the debug image ourselves
# (the ONNX backend has no Ultralytics to call .plot() for us).
_CLASS_COLOURS = ((255, 140, 0), (60, 200, 60), (255, 0, 255), (0, 200, 255))

# Letterbox padding colour (Ultralytics uses 114).
_PAD_VALUE = (114, 114, 114)

# onnxruntime execution providers, most capable first.  TensorRT executes the
# SAME onnx graph, so selecting it never changes the artefact we deploy.
_PROVIDER_ORDER = (
    "TensorrtExecutionProvider",
    "CUDAExecutionProvider",
    "CPUExecutionProvider",
)

# ``provider`` parameter -> concrete provider (None = pick the best available).
_PROVIDER_ALIASES = {
    "auto": None,
    "tensorrt": "TensorrtExecutionProvider",
    "cuda": "CUDAExecutionProvider",
    "cpu": "CPUExecutionProvider",
}

# Default location for the cached TensorRT engine.  Building the engine takes
# minutes, so it is cached and reused across runs.
_DEFAULT_TRT_CACHE = "~/.cache/rs1_perception/tensorrt"


class BackendUnavailable(RuntimeError):
    """Raised when a requested inference backend cannot be constructed."""


@dataclass
class Detection:
    """One detection, expressed in ORIGINAL image pixel coordinates."""

    class_id: int
    class_name: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float


class _Backend:
    """Common interface: run inference and describe how it is configured."""

    name = "?"
    artefact = "?"

    def infer(self, bgr: np.ndarray):
        """Return ``(detections, annotated_bgr_or_None)`` for a BGR frame."""
        raise NotImplementedError

    def describe(self) -> str:
        return f"{self.name} ({self.artefact})"


class UltralyticsBackend(_Backend):
    """The ``pytorch`` backend: a ``.pt`` checkpoint via the Ultralytics API.

    ``ultralytics`` is imported lazily inside ``__init__`` so that the onnx and
    tensorrt backends never pull in torch.
    """

    def __init__(self, node, weights, name, imgsz, device, conf, iou):
        from ultralytics import YOLO  # noqa: PLC0415 - deliberately lazy (pulls in torch)

        self.name = name
        self.artefact = weights
        self.model = YOLO(weights)
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        # 'auto' -> let Ultralytics pick (it prefers CUDA when available).
        self.device = None if device in (None, "", "auto") else device

    def _name_for(self, class_id: int) -> str:
        names = self.model.names
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        try:
            return str(names[class_id])
        except (IndexError, TypeError):
            return str(class_id)

    def infer(self, bgr):
        kwargs = {
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "verbose": False,
        }
        if self.device is not None:
            kwargs["device"] = self.device

        results = self.model(bgr, **kwargs)

        detections: list[Detection] = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                # xyxy -> [x1, y1, x2, y2] in original image coordinates
                coords = box.xyxy[0].to("cpu").detach().numpy().copy()
                class_id = int(box.cls)
                detections.append(
                    Detection(
                        class_id=class_id,
                        class_name=self._name_for(class_id),
                        score=float(box.conf),
                        x1=float(coords[0]),
                        y1=float(coords[1]),
                        x2=float(coords[2]),
                        y2=float(coords[3]),
                    )
                )

        annotated = results[0].plot() if len(results) else None
        return detections, annotated


class OnnxBackend(_Backend):
    """ONNX Runtime backend - deliberately free of any torch dependency.

    Two Ultralytics export flavours are understood, chosen automatically from the
    graph's output shape:

    ``(1, N, 6)``
        The end-to-end export (``nms=True``): one row per detection as
        ``x1, y1, x2, y2, confidence, class_id``, already NMS-filtered.  Simple,
        but NMS *and the confidence floor* are frozen into the graph at export
        time.
    ``(1, 4 + nc, M)``
        The raw detection head (``nms=False``): ``xywh`` boxes plus per-class
        scores, which this backend decodes and NMS-filters itself.  That flavour
        honours ``confidence_threshold`` and ``iou_threshold`` at runtime and so
        behaves like the pytorch backend.

    Either way the graph works in the *letterboxed* input space, so boxes are
    mapped back to original image coordinates and clipped to the frame, matching
    what Ultralytics' ``scale_boxes`` does.
    """

    name = BACKEND_ONNX

    def __init__(
        self,
        node,
        onnx_path,
        imgsz,
        conf,
        iou,
        class_names=None,
        provider="auto",
        trt_cache_dir=_DEFAULT_TRT_CACHE,
        trt_fp16=True,
        backend_name=BACKEND_ONNX,
    ):
        import onnxruntime as ort  # noqa: PLC0415 - the only heavy dep on this path

        if cv2 is None:
            raise BackendUnavailable(
                "the 'onnx' backend needs OpenCV for letterboxing "
                "(`pip install -r requirements/onnx.txt`)"
            )

        self.name = backend_name
        self.artefact = onnx_path
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.provider = str(provider)

        available = ort.get_available_providers()
        if self.provider not in _PROVIDER_ALIASES:
            raise BackendUnavailable(
                f"unknown provider '{self.provider}'; expected one of "
                f"{', '.join(sorted(_PROVIDER_ALIASES))}"
            )
        requested = _PROVIDER_ALIASES[self.provider]
        if requested is not None and requested not in available:
            raise BackendUnavailable(
                f"execution provider '{requested}' is not available in this "
                f"onnxruntime build (available: {', '.join(available)})."
                f"{_provider_hint(requested)}"
            )

        # Preferred provider first, then everything else as a fallback so the
        # session can still be created if the first choice fails to initialise.
        chain = [requested] if requested else [
            p for p in _PROVIDER_ORDER if p in available
        ]
        chain += [p for p in available if p not in chain]

        providers = []
        for name in chain:
            if name == "TensorrtExecutionProvider":
                trt_options = {
                    "trt_engine_cache_enable": True,
                    "trt_timing_cache_enable": True,
                    "trt_fp16_enable": bool(trt_fp16),
                }
                if trt_cache_dir not in (None, "", "none", "off"):
                    cache_dir = os.path.expanduser(str(trt_cache_dir))
                    os.makedirs(cache_dir, exist_ok=True)
                    trt_options["trt_engine_cache_path"] = cache_dir
                providers.append((name, trt_options))
            else:
                providers.append(name)

        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(onnx_path, options, providers=providers)
        self.providers = self.session.get_providers()
        self.input_name = self.session.get_inputs()[0].name

        # An ONNX graph exported with dynamic=False (the default) has a STATIC
        # input size: feeding it anything else fails on every single inference.
        # Catch that here so it is one clear startup message - and so that
        # 'backend: auto' can fall through to a backend that does accept this
        # imgsz - rather than a per-request error that looks like "no trees".
        input_shape = self.session.get_inputs()[0].shape
        graph_hw = tuple(input_shape[2:])
        if all(isinstance(dim, int) for dim in graph_hw) and graph_hw != (
            self.imgsz,
            self.imgsz,
        ):
            raise BackendUnavailable(
                f"{onnx_path} was exported for {graph_hw[0]}x{graph_hw[1]} px input, "
                f"but imgsz is {self.imgsz}. Either set imgsz to {graph_hw[0]}, or "
                f"re-export: `python yolo_training/export_onnx.py --format onnx "
                f"--imgsz {self.imgsz}`"
            )

        shape = tuple(self.session.get_outputs()[0].shape)
        if len(shape) != 3:
            raise BackendUnavailable(
                f"{onnx_path} has output shape {shape}; expected a 3-D "
                "(1, N, 6) end-to-end graph or a (1, 4+nc, M) raw head. "
                "Re-export with `python yolo_training/export_onnx.py`."
            )
        # (1, N, 6) is the end-to-end graph (NMS already applied); anything else
        # with a 3-D shape is treated as the raw head.
        self.end2end = shape[-1] == 6
        self.output_shape = shape

        # Prefer an explicit ROS param, then the class names Ultralytics embeds in
        # the ONNX metadata, and finally a generic fallback.
        self.names = class_names or _names_from_onnx_metadata(self.session)

    def _name_for(self, class_id: int) -> str:
        return self.names.get(class_id, f"class_{class_id}")

    def describe(self) -> str:
        active = self.providers[0] if self.providers else "?"
        return f"{self.name} ({self.artefact}, provider={active})"

    def _decode_head(self, raw: np.ndarray):
        """Decode a raw ``(1, 4+nc, M)`` head into xyxy boxes/scores/class ids."""
        predictions = raw[0].T  # (M, 4 + nc); xywh already in input pixels
        class_scores = predictions[:, 4:]
        class_ids = class_scores.argmax(axis=1)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]

        xywh = predictions[:, :4]
        boxes = np.empty_like(xywh)
        boxes[:, 0] = xywh[:, 0] - xywh[:, 2] / 2.0  # x1
        boxes[:, 1] = xywh[:, 1] - xywh[:, 3] / 2.0  # y1
        boxes[:, 2] = xywh[:, 0] + xywh[:, 2] / 2.0  # x2
        boxes[:, 3] = xywh[:, 1] + xywh[:, 3] / 2.0  # y2

        # Class-aware NMS, matching Ultralytics' default (agnostic_nms=False).
        keep: list[int] = []
        for class_id in np.unique(class_ids):
            candidate = np.flatnonzero(class_ids == class_id)
            remaining = candidate[np.argsort(-scores[candidate])]
            while remaining.size:
                current = remaining[0]
                keep.append(int(current))
                if remaining.size == 1:
                    break
                rest = remaining[1:]
                remaining = rest[_iou(boxes[current], boxes[rest]) <= self.iou]

        keep.sort()
        selected = np.array(keep, dtype=int)
        return boxes[selected], scores[selected], class_ids[selected]

    def infer(self, bgr):
        height, width = bgr.shape[:2]
        tensor, gain, pad_x, pad_y = letterbox(bgr, self.imgsz)
        raw = np.asarray(
            self.session.run(None, {self.input_name: tensor})[0], dtype=np.float32
        )

        if self.end2end:
            rows = raw.reshape(-1, raw.shape[-1])
            boxes, scores = rows[:, :4], rows[:, 4]
            class_ids = np.rint(rows[:, 5]).astype(int)
        else:
            boxes, scores, class_ids = self._decode_head(raw)

        detections: list[Detection] = []
        for i in range(boxes.shape[0]):
            if scores[i] < self.conf:
                continue
            # Undo the letterbox, then clip to the frame (as scale_boxes does).
            x1 = min(max((boxes[i, 0] - pad_x) / gain, 0.0), float(width))
            y1 = min(max((boxes[i, 1] - pad_y) / gain, 0.0), float(height))
            x2 = min(max((boxes[i, 2] - pad_x) / gain, 0.0), float(width))
            y2 = min(max((boxes[i, 3] - pad_y) / gain, 0.0), float(height))
            class_id = int(class_ids[i])
            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=self._name_for(class_id),
                    score=float(scores[i]),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )
        return detections, draw_detections(bgr, detections)


def _iou(box, boxes):
    """IoU of a single ``xyxy`` box against an array of ``xyxy`` boxes."""
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_one = (box[2] - box[0]) * (box[3] - box[1])
    area_many = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    return inter / np.maximum(area_one + area_many - inter, 1e-9)


def letterbox(bgr: np.ndarray, imgsz: int):
    """Resize + pad ``bgr`` to ``imgsz`` x ``imgsz`` the way Ultralytics does.

    Returns ``(tensor, gain, pad_x, pad_y)`` where ``tensor`` is a contiguous
    ``(1, 3, imgsz, imgsz)`` float32 RGB array, ``gain`` is the scale factor and
    ``pad_x``/``pad_y`` are the integer left/top padding in letterboxed pixels.

    Undo it with ``x_original = (x_letterboxed - pad_x) / gain``.
    """
    h, w = bgr.shape[:2]
    gain = min(imgsz / h, imgsz / w)
    new_w, new_h = round(w * gain), round(h * gain)

    resized = bgr if (new_w, new_h) == (w, h) else cv2.resize(
        bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR
    )

    pad_x = (imgsz - new_w) / 2.0
    pad_y = (imgsz - new_h) / 2.0
    left, top = round(pad_x - 0.1), round(pad_y - 0.1)
    right, bottom = imgsz - new_w - left, imgsz - new_h - top

    canvas = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=_PAD_VALUE
    )
    # HWC BGR uint8 -> CHW RGB float32 in [0, 1], batched.
    tensor = canvas[:, :, ::-1].astype(np.float32) / 255.0
    return np.ascontiguousarray(tensor.transpose(2, 0, 1))[None], gain, left, top


def draw_detections(bgr: np.ndarray, detections) -> np.ndarray:
    """Draw the detections onto a copy of ``bgr`` (used by the ONNX backend)."""
    canvas = bgr.copy()
    for det in detections:
        colour = _CLASS_COLOURS[det.class_id % len(_CLASS_COLOURS)]
        p1 = (int(round(det.x1)), int(round(det.y1)))
        p2 = (int(round(det.x2)), int(round(det.y2)))
        cv2.rectangle(canvas, p1, p2, colour, 2)

        label = f"{det.class_name} {det.score:.2f}"
        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(
            canvas,
            (p1[0], max(0, p1[1] - text_h - 6)),
            (p1[0] + text_w + 4, p1[1]),
            colour,
            -1,
        )
        cv2.putText(
            canvas,
            label,
            (p1[0] + 2, max(text_h, p1[1] - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return canvas


def _names_from_onnx_metadata(session) -> dict:
    """Recover the ``names`` mapping Ultralytics embeds in the ONNX metadata."""
    try:
        raw = (session.get_modelmeta().custom_metadata_map or {}).get("names")
    except Exception:  # pragma: no cover - metadata is optional
        return {}
    if not raw:
        return {}
    try:
        parsed = ast.literal_eval(raw)
    except (ValueError, SyntaxError):
        return {}
    if isinstance(parsed, dict):
        return {int(k): str(v) for k, v in parsed.items()}
    if isinstance(parsed, (list, tuple)):
        return {i: str(v) for i, v in enumerate(parsed)}
    return {}


def _swap_suffix(path: str, suffix: str) -> str:
    """``best.pt`` + ``.onnx`` -> ``best.onnx``."""
    for known in (".pt", ".pth", ".onnx", ".engine", ".tflite"):
        if path.endswith(known):
            return path[: -len(known)] + suffix
    return path + suffix


def _resolve_artifact(raw_path) -> str:
    """Resolve a model path, falling back to the package's installed models dir.

    The launch file always passes absolute paths, but running the node directly
    (``ros2 run rs1_perception yolo_node.py``) leaves the parameter defaults
    relative - ``model_path: "best.pt"`` and therefore ``best.onnx`` - which would
    otherwise be looked for in the current working directory instead of in the
    package.  So for a relative path, prefer the CWD only if the file is genuinely
    there, and otherwise use ``<package share>/models/<path>``, the same directory
    the launch file points at.
    """
    path = os.path.expanduser(str(raw_path))
    if os.path.isabs(path) or os.path.isfile(path):
        return path
    try:
        from ament_index_python.packages import get_package_share_directory

        shared = os.path.join(
            get_package_share_directory("rs1_perception"), "models", path
        )
    except Exception:  # noqa: BLE001 - ament index missing: keep the raw path
        return path
    return shared


def _class_names_param(value):
    """ROS string-array params arrive as a list; treat ``['']`` as unset."""
    if not value:
        return None
    names = [str(v) for v in value if str(v) != ""]
    return {i: name for i, name in enumerate(names)} or None


def _provider_hint(provider: str) -> str:
    """Actionable advice for a missing execution provider."""
    if provider == "TensorrtExecutionProvider":
        return (
            " Install the GPU build of onnxruntime and the TensorRT libraries"
            " (`pip install -r requirements/tensorrt.txt`), and make sure the"
            " TensorRT shared libraries are on LD_LIBRARY_PATH."
        )
    if provider == "CUDAExecutionProvider":
        return " Install the GPU build of onnxruntime (`pip install onnxruntime-gpu`)."
    return " Install onnxruntime (`pip install -r requirements/onnx.txt`)."


def _pytorch_backend(node, pt_path, imgsz, device, conf, iou):
    if not os.path.isfile(pt_path):
        raise BackendUnavailable(f"PyTorch weights not found: {pt_path}")
    try:
        return UltralyticsBackend(
            node, pt_path, BACKEND_PYTORCH, imgsz, device, conf, iou
        )
    except ImportError as ex:
        raise BackendUnavailable(
            "the 'pytorch' backend needs ultralytics + torch "
            f"(`pip install -r requirements/pytorch.txt`): {ex}"
        ) from ex


def _onnx_backend(
    node,
    onnx_path,
    imgsz,
    conf,
    iou,
    class_names,
    provider="auto",
    trt_cache_dir=None,
    trt_fp16=True,
    backend_name=BACKEND_ONNX,
):
    if not os.path.isfile(onnx_path):
        raise BackendUnavailable(
            f"ONNX model not found: {onnx_path} (create it with "
            "`python yolo_training/export_onnx.py --format onnx`). "
            f"The '{backend_name}' backend runs this same onnx graph."
        )
    try:
        return OnnxBackend(
            node,
            onnx_path,
            imgsz,
            conf,
            iou,
            class_names,
            provider,
            trt_cache_dir,
            trt_fp16,
            backend_name,
        )
    except ImportError as ex:
        profile = "tensorrt" if backend_name == BACKEND_TENSORRT else "onnx"
        raise BackendUnavailable(
            f"the '{backend_name}' backend needs onnxruntime "
            f"(`pip install -r requirements/{profile}.txt`): {ex}"
        ) from ex


def _tensorrt_backend(node, onnx_path, imgsz, conf, iou, class_names,
                      trt_cache_dir, trt_fp16):
    """TensorRT via onnxruntime's TensorrtExecutionProvider.

    Built on the ONNX artefact rather than an Ultralytics ``.engine`` on purpose:
    one model file to deploy, and TensorRT acceleration that needs neither
    ultralytics nor torch installed.
    """
    return _onnx_backend(
        node,
        onnx_path,
        imgsz,
        conf,
        iou,
        class_names,
        provider="tensorrt",
        trt_cache_dir=trt_cache_dir,
        trt_fp16=trt_fp16,
        backend_name=BACKEND_TENSORRT,
    )


def create_backend(
    node,
    backend,
    artefact_paths,
    imgsz,
    device,
    conf,
    iou,
    class_names,
    provider="auto",
    trt_cache_dir=None,
    trt_fp16=True,
):
    """Build the requested backend, or resolve ``auto`` to the best usable one."""
    log = node.get_logger()
    builders = {
        BACKEND_PYTORCH: lambda: _pytorch_backend(
            node, artefact_paths[BACKEND_PYTORCH], imgsz, device, conf, iou
        ),
        BACKEND_ONNX: lambda: _onnx_backend(
            node,
            artefact_paths[BACKEND_ONNX],
            imgsz,
            conf,
            iou,
            class_names,
            provider,
            trt_cache_dir,
            trt_fp16,
        ),
        BACKEND_TENSORRT: lambda: _tensorrt_backend(
            node,
            artefact_paths[BACKEND_ONNX],
            imgsz,
            conf,
            iou,
            class_names,
            trt_cache_dir,
            trt_fp16,
        ),
    }

    if backend != BACKEND_AUTO:
        if backend not in builders:
            raise BackendUnavailable(
                f"unknown backend '{backend}'; expected one of "
                f"{BACKEND_AUTO}, {BACKEND_PYTORCH}, {BACKEND_ONNX}, {BACKEND_TENSORRT}"
            )
        impl = builders[backend]()
        log.info(f"using '{impl.name}' backend -> {impl.artefact}")
        return impl

    reasons = []
    for candidate in _AUTO_ORDER:
        try:
            impl = builders[candidate]()
        except BackendUnavailable as ex:
            reasons.append(f"{candidate}: {ex}")
            continue
        log.info(
            f"backend=auto selected '{impl.name}' -> {impl.artefact} "
            f"(reason: {'; '.join(reasons) if reasons else 'first choice available'})"
        )
        return impl

    raise BackendUnavailable(
        "no usable inference backend found:\n  " + "\n  ".join(reasons)
    )


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('image_topic', '/parrot1/camera/image')
        # 'auto' | 'pytorch' | 'onnx' | 'tensorrt' (see the module docstring)
        self.declare_parameter('backend', BACKEND_AUTO)
        # PyTorch weights; the onnx/tensorrt backends default to the same stem.
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('onnx_model_path', '')
        # Which onnxruntime execution provider the onnx/tensorrt backends prefer.
        # 'auto' | 'tensorrt' | 'cuda' | 'cpu'
        self.declare_parameter('provider', 'auto')
        # Where the TensorRT engine built from the onnx graph is cached.  Building
        # it takes minutes, so it is reused across runs.  '' or 'off' disables it.
        self.declare_parameter('trt_cache_dir', _DEFAULT_TRT_CACHE)
        self.declare_parameter('trt_fp16', True)
        # Optional class-name override; [''] / [] means "read from the model".
        self.declare_parameter('class_names', [''])
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')

        self.image_topic = self.get_parameter('image_topic').value
        requested_backend = str(self.get_parameter('backend').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        device = str(self.get_parameter('device').value)
        provider = str(self.get_parameter('provider').value)
        trt_cache_dir = str(self.get_parameter('trt_cache_dir').value)
        trt_fp16 = bool(self.get_parameter('trt_fp16').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        class_names = _class_names_param(self.get_parameter('class_names').value)

        # Resolve the artefacts.  Relative paths (i.e. the parameter defaults) fall
        # back to the package's installed share/models directory, so running the
        # node directly works from any working directory as well as via the launch
        # file.  Note there are only two model files: the .pt for the pytorch
        # backend, and the .onnx that both the onnx and tensorrt backends execute.
        pt_path = _resolve_artifact(self.get_parameter('model_path').value)
        declared_onnx = str(self.get_parameter('onnx_model_path').value)
        onnx_path = (
            _resolve_artifact(declared_onnx)
            if declared_onnx
            else _swap_suffix(pt_path, '.onnx')
        )

        self.backend = create_backend(
            self,
            requested_backend,
            {
                BACKEND_PYTORCH: pt_path,
                BACKEND_ONNX: onnx_path,
            },
            self.imgsz,
            device,
            self.confidence_threshold,
            iou_threshold,
            class_names,
            provider,
            trt_cache_dir,
            trt_fp16,
        )

        self.latest_image = None
        self.latest_image_header = None

        self.parrot_cam_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.detect_service = self.create_service(
            DetectYolo, '/yolo/detect', self.detect_callback
        )

        self.yolo_img_pub = self.create_publisher(Image, self.debug_image_topic, 1)

        self.get_logger().info(
            f"yolo_node ready (image_topic={self.image_topic}, "
            f"backend={self.backend.describe()}, imgsz={self.imgsz}, "
            f"conf={self.confidence_threshold})"
        )

    @staticmethod
    def _image_to_bgr(msg):
        if msg.encoding == "bgr8":
            return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ).copy()
        if msg.encoding == "rgb8":
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ).copy()
            return arr[:, :, ::-1].copy()  # RGB -> BGR
        if msg.encoding in ("mono8", "8UC1"):
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width
            ).copy()
            return np.repeat(arr[:, :, None], 3, axis=2)
        return None

    @staticmethod
    def _bgr_to_image(bgr, header):
        arr = np.ascontiguousarray(bgr, dtype=np.uint8)
        msg = Image()
        msg.header = header
        msg.height = int(arr.shape[0])
        msg.width = int(arr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(arr.shape[1] * 3)
        msg.data = arr.tobytes()
        return msg

    def image_callback(self, msg):
        try:
            image = self._image_to_bgr(msg)
        except Exception as ex:
            self.get_logger().warn(f"Failed to decode image: {ex}")
            return
        if image is None:
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return
        self.latest_image = image
        self.latest_image_header = msg.header

    def detect_callback(self, request, response):
        detections = YoloDetections()
        detections.header.frame_id = "parrot1_camera_rgb_optical_frame"
        detections.header.stamp = self.get_clock().now().to_msg()
        response.detections = detections

        if not request.start_yolo:
            self.get_logger().warn("DetectYolo called with start_yolo=false; returning no detections")
            return response

        if self.latest_image is None:
            self.get_logger().warn("No camera image received yet; returning no detections")
            return response

        try:
            found, annotated_frame = self.backend.infer(self.latest_image)
        except Exception as ex:  # noqa: BLE001 - keep the service responsive
            self.get_logger().error(
                f"Inference failed on the '{self.backend.name}' backend: {ex}"
            )
            return response

        for det in found:
            bounding_box = BoundingBox()
            bounding_box.class_name = det.class_name
            bounding_box.score = det.score
            bounding_box.left = int(det.x1)
            bounding_box.top = int(det.y1)
            bounding_box.right = int(det.x2)
            bounding_box.bottom = int(det.y2)
            detections.bounding_boxes.append(bounding_box)

        if self.publish_debug_image and annotated_frame is not None:
            self.yolo_img_pub.publish(self._bgr_to_image(annotated_frame, detections.header))

        self.get_logger().info(
            f"[{self.backend.name}] Detected {len(detections.bounding_boxes)} tree(s)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloNode()
    except BackendUnavailable as ex:
        # A backend/configuration problem (e.g. tensorrt requested on a machine
        # with no NVIDIA GPU, or a missing .onnx) should be one clear message
        # rather than a traceback.
        rclpy.logging.get_logger('yolo_node').fatal(
            f"{ex}\nSee rs1_ws/src/rs1_perception/README.md for the per-backend "
            "requirements."
        )
        rclpy.shutdown()
        raise SystemExit(1) from None

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or the context being shut down by something else: not an error.
        pass
    finally:
        # Guarded because after an external shutdown the context is already gone
        # and calling destroy_node()/shutdown() again raises RCLError.
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()