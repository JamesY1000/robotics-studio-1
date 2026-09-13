#!/usr/bin/env python3
"""Camera-based YOLO sample collector.

Collects RGB images from the Parrot drone's camera together with the 3D
ground-truth boxes published by the ``ObjectLabelPlugin`` (see the tree models
in ``rs1_ws/src/rs1_sim/models/trees``), projects each 3D box into the image and
stores the result in YOLO format::

    <class_id> <cx> <cy> <w> <h>     (all normalised 0..1)

It is driven by ``sample_generator.py`` through the same ROS services as the
lidar-based ``sample_collector.py`` (CollectSample / UpdateGtSubs /
CheckModelsLoaded / GetCurrentSampleId / GetGtData), so the existing generator
and ``model_importer.py`` work unchanged.

Self-contained: only depends on rclpy + tf2_ros + cv2 + numpy (no
``arculus_common`` / ``brain3_common``).
"""

from functools import partial
from typing import Any, Dict, List, Optional, Tuple
from pathlib import Path
from dataclasses import dataclass, field

import numpy as np
import cv2
import rclpy
import rclpy.qos
from rclpy.duration import Duration
from rclpy.time import Time as ROSTime
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty, Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_matrix

from sample_gen.base import BaseNode, BaseParams
from sample_gen.services import (
    SERVICE_COLLECT_SAMPLE,
    SERVICE_UNREGISTER_GT_SUBS,
    SERVICE_UPDATE_GT_SUBS,
    SERVICE_CHECK_MODELS_LOADED,
    SERVICE_GET_CURRENT_SAMPLE_ID,
    SERVICE_GET_GT_DATA,
)
from sample_gen.helpers import load_yaml_entry
from sample_gen_msgs.msg import ObjectLabel
from sample_gen_msgs.srv import CollectSample, CheckModelsLoaded, GetCurrentSampleID, GetDBData

NODE_NAME = "camera_collector_node"
SAMPLE_GEN_PKG = "sample_gen"
NEAR_CLIP = 0.05          # m, in front of the camera
# Trees beyond this depth don't reliably render in the rgbd image (the declared
# 40 m clip still leaves faint/smeared far trees), so cull them conservatively.
FAR_CLIP = 30.0           # m
MIN_BOX_PIXELS = 8        # skip tiny projected boxes (noise)
# A tree is only included if at least this fraction of its projected box lies
# inside the image frame. 0.15 keeps trees at the frame edge (still useful for
# YOLO) and only drops ones that are almost entirely off-screen.
MIN_FRAME_VISIBLE_FRACTION = 0.15

# The request stamp comes from the ROS clock, which is bridged from Gazebo with
# a small lag (a few ms). A camera frame / tree TF captured *just before* the
# models finished moving can therefore still carry a stamp >= the request stamp
# while showing the PREVIOUS positions, so some boxes land one sample ahead of
# the image. Require the image and TFs to be at least this much NEWER than the
# request stamp before accepting them. 1 s (sim time) is far larger than the
# bridge lag + several camera frames, so the data is guaranteed post-move.
FRESHNESS_MARGIN_NS = 2_000_000_000  # 2 s (sim time)

# X/Y size scale per classification id. Pines (class 2) are sparse cones, so 0.8
# of the 3.17 m AABB keeps the foliage in the box without being oversized for
# close pines. Oaks (class 1) have a 10.6 x 8.5 m collision AABB but their dense
# canopy is only ~30% of the X span, so 0.4 keeps the box snug around the foliage.
BBOX_XY_SCALE = {1: 0.4, 2: 0.8}
BBOX_Z_SCALE = {1: 0.85, 2: 1.0}

# A tree whose projected box is more than this fraction covered by NEARER trees'
# projected boxes is considered mostly hidden and is dropped from the labels.
# 0.6 drops trees that are mostly covered by a nearer tree (e.g. an oak behind a
# pine) while still keeping trees that are only partially overlapped.
OCCLUSION_SKIP_FRACTION = 0.6

# Visibility check constants. A tree box must contain a minimum fraction of dark
# (foliage/trunk) pixels, otherwise it is over sky/empty and the tree is not
# rendered there. The depth image additionally drops trees occluded by a nearer
# surface.
MIN_DARK_FRACTION = 0.03
DARK_THRESHOLD = 100.0        # BGR mean below this counts as foliage/trunk
DEPTH_OCCLUSION_MARGIN = 1.5  # m, depth must be this much closer to be occluding


def _resume_sample_id(db_path: Path) -> int:
    """Next sample id to use, continuing from an existing sample_database.csv.

    Returns 0 for a fresh or empty database, otherwise the highest saved sample
    id plus one (so a crashed run can be relaunched without overwriting the
    samples already collected).
    """
    if not db_path.is_file():
        return 0
    max_id = -1
    try:
        with open(db_path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                first = line.split(",")[0]
                if first == "ID" or not first.isdigit():
                    continue  # header or malformed line
                max_id = max(max_id, int(first))
    except Exception:
        return 0
    return max_id + 1


@dataclass
class CameraCollectorParams(BaseParams):
    database_dir: str = "/tmp/sample_gen/database"
    visualize: bool = False
    image_topic: str = "/parrot1/camera/image"
    depth_image_topic: str = "/parrot1/camera/depth/image"
    camera_info_topic: str = ""
    hfov: float = 2.0944  # Parrot rgbd camera HFOV (rad)
    frame_id: str = "world"
    camera_frame: str = "parrot1_camera_rgb_optical_frame"
    # Vertical shift (world Z, metres) applied to oak boxes to centre them on
    # the dense canopy rather than the full collision AABB. Defaults to 0 (the
    # original bringup flow); the forest launch overrides this to ~0.65.
    oak_z_center_offset: float = 0.0
    class_config_yml: str = "class_configs.yml"
    class_config_name: str = "class_config"
    class_config_dict_name: str = "classes"


class CameraCollector(BaseNode):
    def __init__(self):
        self._params: CameraCollectorParams = CameraCollectorParams()  # pyright: ignore [reportIncompatibleVariableOverride]
        super().__init__(NODE_NAME)

        self.class_config = load_yaml_entry(
            self._params.class_config_name, SAMPLE_GEN_PKG, self._params.class_config_yml
        )
        if self.class_config:
            self.class_config_dict = self.class_config[self._params.class_config_dict_name]
            self.class_type_dict = {x["id"]: x["class"] for x in self.class_config_dict if "id" and "class" in x}
        else:
            self.get_logger().error("Class config parameter is empty, node will not be able to map class ids")
            self.destroy_node()
            return

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Latest camera data
        self.image_msg: Optional[Image] = None
        self.camera_info_msg: Optional[CameraInfo] = None
        self.K: Optional[np.ndarray] = None

        # Ground-truth subscribers (one per spawned model)
        self.gt_subs = []
        self.obj_labels: List[list] = []
        self._init_image_subs()
        self._update_gt_subs()

        # Output directories
        self.db_dir = Path(self._params.database_dir, "camera_samples")
        self.images_dir = Path(self.db_dir, "images")
        self.labels_dir = Path(self.db_dir, "labels")
        self.labelled_dir = Path(self.db_dir, "labelled")
        for d in (self.images_dir, self.labels_dir, self.labelled_dir):
            d.mkdir(parents=True, exist_ok=True)
        self.db_path = Path(self.db_dir, "sample_database.csv")
        # Resume support: continue from the last saved sample so a crashed run
        # can be relaunched without overwriting what was already collected.
        self.cur_sample_id = _resume_sample_id(self.db_path)
        self._wrote_header = self.db_path.is_file() and self.db_path.stat().st_size > 0

        # Services
        self.collect_sample_srv = self.create_service(
            CollectSample, SERVICE_COLLECT_SAMPLE, self._collect_sample_cb
        )
        self.unregister_gt_subs_srv = self.create_service(
            Empty, SERVICE_UNREGISTER_GT_SUBS, self._unregister_gt_subs_cb
        )
        self.update_gt_subs_srv = self.create_service(
            Trigger, SERVICE_UPDATE_GT_SUBS, self._update_gt_subs_cb
        )
        self.check_models_loaded_srv = self.create_service(
            CheckModelsLoaded, SERVICE_CHECK_MODELS_LOADED, self._check_models_loaded_cb
        )
        self.get_current_sample_id_srv = self.create_service(
            GetCurrentSampleID, SERVICE_GET_CURRENT_SAMPLE_ID, self._get_current_sample_id_cb
        )
        self.get_gt_data_srv = self.create_service(
            GetDBData, SERVICE_GET_GT_DATA, self._get_gt_data_cb
        )
        self.get_logger().info("Initialized")

    # ------------------------------------------------------------------ subs
    def _init_image_subs(self) -> None:
        self.create_subscription(
            Image, self._params.image_topic, self._image_cb, qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to camera image on {self._params.image_topic}")
        self.depth_msg: Optional[Image] = None
        if self._params.depth_image_topic:
            self.create_subscription(
                Image, self._params.depth_image_topic, self._depth_cb, qos_profile=rclpy.qos.qos_profile_sensor_data
            )
            self.get_logger().info(f"Subscribed to depth image on {self._params.depth_image_topic}")
        if self._params.camera_info_topic:
            self.create_subscription(
                CameraInfo,
                self._params.camera_info_topic,
                self._camera_info_cb,
                qos_profile=rclpy.qos.qos_profile_sensor_data,
            )
            self.get_logger().info(f"Subscribed to camera info on {self._params.camera_info_topic}")

    def _image_cb(self, msg: Image) -> None:
        self.image_msg = msg
        # Precompute intrinsics from hfov + image size if no camera_info yet
        if self.K is None:
            self.K = self._intrinsics_from_hfov(msg.width, msg.height)

    def _depth_cb(self, msg: Image) -> None:
        self.depth_msg = msg

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info_msg = msg
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.get_logger().info(f"Got camera info K={self.K.tolist()}")

    def _intrinsics_from_hfov(self, width: int, height: int) -> np.ndarray:
        fx = (width / 2.0) / np.tan(self._params.hfov / 2.0)
        fy = fx  # square pixels
        return np.array([[fx, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]])

    # ----------------------------------------------------------- GT services
    def _update_gt_subs(self) -> None:
        self._unregister_gt_subs()
        self._init_object_label_subs()

    def _update_gt_subs_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._update_gt_subs()
        response.success = True
        response.message = ""
        return response

    def _unregister_gt_subs_cb(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self._unregister_gt_subs()
        return response

    def _unregister_gt_subs(self) -> None:
        if self.gt_subs:
            for sub in self.gt_subs:
                self.destroy_subscription(sub)
            self.gt_subs.clear()
            self.obj_labels.clear()

    def _init_object_label_subs(self) -> None:
        try:
            topics = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f"Couldn't get published topics: {e}")
            return

        self.obj_labels.clear()
        self.gt_subs.clear()
        idx = 0
        for topic_name, topic_types in topics:
            if "sample_gen_msgs/msg/ObjectLabel" in topic_types:
                cb = partial(self._object_label_cb, idx)
                self.gt_subs.append(self.create_subscription(ObjectLabel, topic_name, cb, 1))
                self.obj_labels.append([None, topic_name.split("/")[-1]])
                idx += 1
        self.get_logger().info(f"Subscribed to {len(self.obj_labels)} ObjectLabel topics")

    def _object_label_cb(self, idx: int, msg: ObjectLabel) -> None:
        self.obj_labels[idx][0] = msg

    # ------------------------------------------------------------ TF helpers
    def _get_trafo(self, parent_frame: str, child_frame: str, stamp: Optional[ROSTime] = None) -> Optional[Any]:
        """Transform that moves points from ``child_frame`` into ``parent_frame``.

        ``stamp=None`` (or zero) means "latest available" transform - avoids
        clock-base mismatches between the request stamp and sensor stamps.
        """
        if stamp is None or stamp.nanoseconds == 0:
            stamp = ROSTime()  # zero time = latest available in tf2
        try:
            return self.tf_buffer.lookup_transform(
                parent_frame, child_frame, stamp, timeout=Duration(seconds=1.0)
            )
        except Exception as ex:
            self.get_logger().warn(f"Could not transform {child_frame} to {parent_frame}: {ex}")
            return None

    def _transform_points(self, points: np.ndarray, trafo: Any) -> np.ndarray:
        """Transform points from the transform's child frame into its parent frame.

        ``lookup_transform(parent, child)`` maps a point with
        ``p_parent = R @ p_child + t`` (R = child->parent rotation, t = child
        origin in parent frame). ``points @ R.T`` applies R to each row, so the
        correct expression is ``points @ R.T + t``.
        """
        q = trafo.transform.rotation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        t = np.array([trafo.transform.translation.x, trafo.transform.translation.y, trafo.transform.translation.z])
        return points @ R.T + t

    def _world_center_and_yaw(self, label: ObjectLabel, stamp: Optional[ROSTime] = None) -> Optional[Tuple[np.ndarray, float]]:
        """Return bbox centre in world frame and yaw (from world->model TF)."""
        if label.header.frame_id.replace("/", "") == self._params.frame_id.replace("/", ""):
            pos = label.pose.position
            return np.array([pos.x, pos.y, pos.z]), 0.0
        # model -> world transform (latest if stamp is None)
        trafo = self._get_trafo(self._params.frame_id, label.header.frame_id, stamp)
        if trafo is None:
            return None
        pos = label.pose.position
        centre = self._transform_points(np.array([[pos.x, pos.y, pos.z]]), trafo)[0]
        q = trafo.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return centre, yaw

    # ------------------------------------------------------- GT aggregation
    def _get_gt_boxes(self, stamp: Optional[ROSTime] = None) -> List[Dict[str, Any]]:
        """Build a list of 3D boxes for all currently-publishing models.

        Labels that have no live data yet (None) and models whose world TF is
        unavailable are skipped - so phantom ObjectLabel topics are harmless.
        """
        boxes = []
        for label_gt, label_name in self.obj_labels:
            if not label_gt:
                continue
            result = self._world_center_and_yaw(label_gt, stamp)
            if result is None:
                continue
            centre, yaw = result
            scale = BBOX_XY_SCALE.get(int(label_gt.classification), 1.0)
            z_scale = BBOX_Z_SCALE.get(int(label_gt.classification), 1.0)
            z_offset = self._params.oak_z_center_offset if int(label_gt.classification) == 1 else 0.0
            size = np.array(
                [label_gt.size.x * scale, label_gt.size.y * scale, label_gt.size.z * z_scale]
            )
            centre = centre + np.array([0.0, 0.0, z_offset])
            boxes.append(
                {
                    "name": label_name,
                    "classification": int(label_gt.classification),
                    "type": self.class_type_dict.get(int(label_gt.classification), "UNCLASSIFIED"),
                    "centre": centre,
                    "size": size,
                    "yaw": yaw,
                    "offset": np.array([label_gt.offset_x, label_gt.offset_y, label_gt.offset_z]),
                    "label": label_gt,
                }
            )
        return boxes

    # ---------------------------------------------------------------- camera
    def _image_to_np(self, msg: Image) -> Optional[np.ndarray]:
        try:
            if msg.encoding == "rgb8":
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == "mono8":
                return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)
            else:
                self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
                return None
        except Exception as e:
            self.get_logger().warn(f"Could not decode image: {e}")
            return None

    def _decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        """Depth image in metres, or None if the encoding is unsupported."""
        try:
            if msg.encoding == "32FC1":
                return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
            if msg.encoding in ("16UC1", "mono16"):
                arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
                return arr.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().warn(f"Could not decode depth image: {e}")
        return None

    def _tree_visible(self, proj: List[float], box_depth: float, img_np: np.ndarray) -> bool:
        """True if the box actually shows the tree (not sky/empty/occluded).

        Checks the CLIPPED box region (not just the centre, which can land on
        sky for a close tree whose canopy pokes above the frame): it must hold
        enough dark foliage pixels, and the depth image must not show a surface
        clearly closer than the tree.
        """
        img_h, img_w = img_np.shape[:2]
        x1 = int(np.clip((proj[0] - proj[2] / 2.0) * img_w, 0, img_w - 1))
        y1 = int(np.clip((proj[1] - proj[3] / 2.0) * img_h, 0, img_h - 1))
        x2 = int(np.clip((proj[0] + proj[2] / 2.0) * img_w, 0, img_w - 1))
        y2 = int(np.clip((proj[1] + proj[3] / 2.0) * img_h, 0, img_h - 1))
        region = img_np[y1:y2 + 1, x1:x2 + 1]
        if region.size == 0:
            return True

        # Dark-foliage check: a tree box must contain some dark pixels.
        brightness = region.mean(axis=2)
        if float((brightness < DARK_THRESHOLD).mean()) < MIN_DARK_FRACTION:
            return False

        # Depth check: a nearer surface occludes the tree.
        if self.depth_msg is not None:
            depth_m = self._decode_depth(self.depth_msg)
            if depth_m is not None and depth_m.shape[:2] == (img_h, img_w):
                dregion = depth_m[y1:y2 + 1, x1:x2 + 1]
                valid = dregion[np.isfinite(dregion) & (dregion > 0.0)]
                if valid.size and float(np.median(valid)) < box_depth - DEPTH_OCCLUSION_MARGIN:
                    return False
        return True

    @staticmethod
    def _clip_box_near(corners_cam: np.ndarray, near: float) -> Optional[np.ndarray]:
        """Clip a box's corners against the near plane (z > near).

        Returns the visible vertices (front corners + edge/plane intersection
        points), or None if nothing is in front of the plane.
        """
        edges = [
            (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
            (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
        ]
        verts: List[np.ndarray] = []
        for i, j in edges:
            p = corners_cam[i]
            q = corners_cam[j]
            p_in = p[2] > near
            q_in = q[2] > near
            if p_in:
                verts.append(p)
            if q_in:
                verts.append(q)
            if p_in != q_in:
                t = (near - p[2]) / (q[2] - p[2])
                r = p + t * (q - p)
                r[2] = near
                verts.append(r)
        if not verts:
            return None
        return np.array(verts, dtype=np.float64)

    def _project_box(self, box: Dict[str, Any], T_cam_world: Any, img_w: int, img_h: int):
        """Project a 3D box into the image.

        Returns ``(proj, corners_px)`` or ``None``. ``proj`` is
        ``[cx, cy, w, h]`` normalised; ``corners_px`` is an Nx2 float32 array of
        the (unclipped) projected corner pixel coords, used for occlusion.
        """
        centre = box["centre"]
        l, w, h = box["size"]
        yaw = box["yaw"]

        # 8 corners of the box in world frame
        hs = np.array([l / 2.0, w / 2.0, h / 2.0])
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        corners_local = signs * hs

        # rotate by yaw around Z (only x/y)
        cy_, sy_ = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy_, -sy_, 0.0], [sy_, cy_, 0.0], [0.0, 0.0, 1.0]])
        corners_world = corners_local @ Rz.T + centre

        # to camera optical frame
        corners_cam = self._transform_points(corners_world, T_cam_world)

        # Optical frame: +Z forward, +X right, +Y down
        z = corners_cam[:, 2]
        # Skip boxes entirely behind the camera...
        if np.all(z <= NEAR_CLIP):
            return None
        # ...or entirely beyond the far clip (nothing renders past it).
        if np.all(z >= FAR_CLIP):
            return None

        # Clip the box against the near plane so a close tree whose box extends
        # behind the camera is still labelled by its visible part (previously
        # np.any(z <= NEAR_CLIP) dropped the whole box for close trees).
        verts = self._clip_box_near(corners_cam, NEAR_CLIP)
        if verts is None or verts.shape[0] == 0:
            return None

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        zc = verts[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = fx * verts[:, 0] / zc + cx
            v = fy * verts[:, 1] / zc + cy

        corners_px = np.column_stack([u, v]).astype(np.float32)

        umin, umax = float(u.min()), float(u.max())
        vmin, vmax = float(v.min()), float(v.max())

        # Clamp to image and check there is any meaningful overlap with the
        # frame. (No "centre must be inside the image" check: a large box -
        # e.g. a close oak - can overlap the frame substantially while its
        # centre lies outside, and those trees ARE partially visible.)
        w_pix = min(umax, img_w - 1) - max(umin, 0.0)
        h_pix = min(vmax, img_h - 1) - max(vmin, 0.0)
        if w_pix <= MIN_BOX_PIXELS or h_pix <= MIN_BOX_PIXELS:
            return None

        # Require at least MIN_FRAME_VISIBLE_FRACTION of the projected box to be
        # inside the frame. A tree whose box is mostly off-screen (e.g. only a
        # sliver visible at an image edge) is dropped rather than labelled.
        total_area = (umax - umin) * (vmax - vmin)
        visible_area = w_pix * h_pix
        if total_area > 0 and visible_area / total_area < MIN_FRAME_VISIBLE_FRACTION:
            return None

        # Clip the box edges to the image frame so a box that extends past the
        # border is reported with its edges along the frame, not beyond it.
        umin_c = np.clip(umin, 0.0, img_w - 1)
        umax_c = np.clip(umax, 0.0, img_w - 1)
        vmin_c = np.clip(vmin, 0.0, img_h - 1)
        vmax_c = np.clip(vmax, 0.0, img_h - 1)

        # Normalise for YOLO (0..1)
        cx_norm = ((umin_c + umax_c) / 2.0) / img_w
        cy_norm = ((vmin_c + vmax_c) / 2.0) / img_h
        w_norm = (umax_c - umin_c) / img_w
        h_norm = (vmax_c - vmin_c) / img_h
        return [cx_norm, cy_norm, w_norm, h_norm], corners_px

    # ------------------------------------------------------------- occlusion
    @staticmethod
    def _corners_to_poly(corners_px: np.ndarray) -> np.ndarray:
        """Convex hull (Nx2 float32) of a box's projected corner pixels."""
        hull = cv2.convexHull(corners_px.reshape(-1, 1, 2))
        return hull.reshape(-1, 2).astype(np.float32)

    def _occlusion_fraction(self, poly: np.ndarray, nearer_polys: List[np.ndarray]) -> float:
        """Fraction of ``poly`` (a projected box's convex hull) covered by
        ``nearer_polys`` (projected hulls of trees closer to the camera).

        Uses the actual projected polygons (not axis-aligned rectangles) so two
        trees that merely overlap in their loose bounding rects are NOT treated
        as occluding each other - only real polygon overlap counts.
        """
        area = cv2.contourArea(poly)
        if area <= 0:
            return 1.0
        covered = 0.0
        for npoly in nearer_polys:
            try:
                inter_area, _ = cv2.intersectConvexConvex(poly, npoly)
                covered += float(inter_area)
            except Exception:
                continue
        return min(covered / area, 1.0)

    # ------------------------------------------------------------ check data
    def _check_new_data(self, stamp: ROSTime) -> bool:
        if self.image_msg is None:
            self.get_logger().info("No image received yet")
            return False
        if self.K is None:
            self.get_logger().info("No camera intrinsics available yet")
            return False
        dt = stamp - ROSTime.from_msg(self.image_msg.header.stamp)
        if dt.nanoseconds > 0:
            self.get_logger().info(f"Image not up to date (dt={dt.nanoseconds}ns)")
            return False
        for label_gt, _ in self.obj_labels:
            if label_gt is None:
                self.get_logger().info("Ground truth not initialized")
                return False
            dt = stamp - ROSTime.from_msg(label_gt.header.stamp)
            if dt.nanoseconds > 0:
                self.get_logger().info("Ground truth not up to date")
                return False
        return True

    # ---------------------------------------------------------------- srv cb
    def _get_current_sample_id_cb(
        self, request: GetCurrentSampleID.Request, response: GetCurrentSampleID.Response
    ) -> GetCurrentSampleID.Response:
        response.sample_id = self.cur_sample_id
        return response

    def _get_gt_data_cb(self, request: GetDBData.Request, response: GetDBData.Response) -> GetDBData.Response:
        # size/offset/classification come straight from the ObjectLabel message -
        # no TF lookup is needed here (the world transform isn't required to
        # measure a model's bounding box, and it avoids stale-stamp TF errors).
        #
        # Pick the most recently published label: after deleting the previous
        # model its ObjectLabel topic can linger briefly in the ROS graph (DDS
        # unadvertisement is async), so "first found" can return stale data from
        # the previous model. The currently-spawned model always has the newest
        # timestamp.
        best = None
        best_time: Optional[ROSTime] = None
        for label_gt, _ in self.obj_labels:
            if label_gt is None:
                continue
            ts = ROSTime.from_msg(label_gt.header.stamp)
            if best_time is None or ts > best_time:
                best_time = ts
                best = label_gt

        if best is None:
            response.success = False
            response.message = "No ground truth data available"
            return response

        response.success = True
        response.size.x = float(best.size.x)
        response.size.y = float(best.size.y)
        response.size.z = float(best.size.z)
        response.offset.x = float(best.offset_x)
        response.offset.y = float(best.offset_y)
        response.offset.z = float(best.offset_z)
        response.classification = int(best.classification)
        response.class_name = self.class_type_dict.get(int(best.classification), "UNCLASSIFIED")
        return response

    def _check_models_loaded_cb(
        self, request: CheckModelsLoaded.Request, response: CheckModelsLoaded.Response
    ) -> CheckModelsLoaded.Response:
        boxes = self._get_gt_boxes()
        if len(boxes) == request.num_models:
            response.success = True
        else:
            response.success = False
            response.message = f"Found {len(boxes)} models, expected: {request.num_models}"
        return response

    def _collect_sample_cb(
        self, request: CollectSample.Request, response: CollectSample.Response
    ) -> CollectSample.Response:
        # The request stamp is the time the models finished moving. The camera
        # frame must have been captured AFTER that, otherwise it still shows the
        # trees at their previous positions. The generator keeps the stamp fixed
        # and retries, so we simply wait for a fresh frame here.
        if self.image_msg is None:
            response.success = False
            response.message = "No image received yet"
            return response
        img_stamp = ROSTime.from_msg(self.image_msg.header.stamp)
        req_stamp = ROSTime.from_msg(request.stamp)
        # The request stamp is the moment the models finished moving. The image
        # must have been CAPTURED after that, otherwise it shows the trees at
        # their previous positions. A margin is added because the ROS clock lags
        # the Gazebo sensor timestamps by a few ms - without it a frame captured
        # just before the move can slip through and be paired with post-move TFs.
        if img_stamp.nanoseconds < req_stamp.nanoseconds + FRESHNESS_MARGIN_NS:
            response.success = False
            response.message = "Image not up to date"
            return response

        # The tree TFs are published by the ObjectLabelPlugin at 10 Hz, slower
        # than the 30 Hz camera - so right after the models move, a fresh image
        # can still be paired with STALE tree TFs. Require each tree's TF to
        # have been published well after the move (stamp >= request + margin).
        for label_gt, _ in self.obj_labels:
            if label_gt is None:
                continue
            frame = label_gt.header.frame_id
            if frame.replace("/", "") == self._params.frame_id.replace("/", ""):
                continue  # label is already in world frame, no TF needed
            try:
                tf = self.tf_buffer.lookup_transform(self._params.frame_id, frame, ROSTime())
            except Exception:
                response.success = False
                response.message = f"Tree TF unavailable for {frame}"
                return response
            if ROSTime.from_msg(tf.header.stamp).nanoseconds < req_stamp.nanoseconds + FRESHNESS_MARGIN_NS:
                response.success = False
                response.message = "Tree TF not up to date"
                return response

        img_np = self._image_to_np(self.image_msg)
        if img_np is None:
            response.success = False
            response.message = "Could not decode image"
            return response
        img_h, img_w = img_np.shape[:2]

        # Camera world pose: transform that maps world -> camera optical frame
        T_cam_world = self._get_trafo(self._params.camera_frame, self._params.frame_id)
        if T_cam_world is None:
            response.success = False
            response.message = f"Could not get TF {self._params.camera_frame} -> {self._params.frame_id}"
            return response

        boxes = self._get_gt_boxes()

        # Project all boxes, nearest first (depth = camera-frame Z of the box
        # centre), so occlusion can be checked against nearer trees only.
        projected = []
        for box in boxes:
            result = self._project_box(box, T_cam_world, img_w, img_h)
            if result is None:
                continue
            proj, corners_px = result
            centre_cam = self._transform_points(np.array([box["centre"]]), T_cam_world)[0]
            projected.append((box, proj, float(centre_cam[2]), corners_px))
        projected.sort(key=lambda item: item[2])

        # Drop boxes that do not actually show the tree in the image: either the
        # centre is sky-bright (not rendered) or the depth image shows a nearer
        # surface occluding it (leaves / another tree / a hill).
        visible = []
        for box, proj, depth, corners_px in projected:
            if self._tree_visible(proj, depth, img_np):
                visible.append((box, proj, depth, corners_px))
            else:
                self.get_logger().info(f"Skipping not-visible tree {box['name']}")
        projected = visible

        # Drop trees that are mostly hidden behind a nearer tree, judged by the
        # overlap of their projected convex hulls (not the loose bounding rects,
        # which previously removed trees that only overlapped in 2D).
        kept = []
        for box, proj, depth, corners_px in projected:
            poly = self._corners_to_poly(corners_px)
            nearer_polys = [self._corners_to_poly(c) for _b, _p, _d, c in kept]
            if self._occlusion_fraction(poly, nearer_polys) >= OCCLUSION_SKIP_FRACTION:
                self.get_logger().info(f"Skipping occluded tree {box['name']}")
                continue
            kept.append((box, proj, depth, corners_px))

        # Project all boxes
        yolo_lines: List[str] = []
        labelled = img_np.copy()
        for box, proj, _depth, _corners in kept:
            cx_n, cy_n, w_n, h_n = proj
            yolo_lines.append(f"{box['classification']} {cx_n:.6f} {cy_n:.6f} {w_n:.6f} {h_n:.6f}")
            if self._params.visualize:
                x1 = int((cx_n - w_n / 2) * img_w)
                y1 = int((cy_n - h_n / 2) * img_h)
                x2 = int((cx_n + w_n / 2) * img_w)
                y2 = int((cy_n + h_n / 2) * img_h)
                cv2.rectangle(labelled, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(labelled, box["type"], (x1, max(y1 - 5, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Save
        sample_name = f"{self.cur_sample_id:06d}"
        img_path = Path(self.images_dir, sample_name + ".jpg")
        label_path = Path(self.labels_dir, sample_name + ".txt")
        cv2.imwrite(str(img_path), img_np)
        with open(label_path, "w") as f:
            f.writelines(line + "\n" for line in yolo_lines)

        if self._params.visualize:
            cv2.imwrite(str(Path(self.labelled_dir, sample_name + ".jpg")), labelled)

        self.get_logger().info(
            f"Saved sample {self.cur_sample_id} ({len(yolo_lines)} objects) to {self.db_dir}"
        )

        # Store database entry
        with open(self.db_path, "a") as f:
            if not self._wrote_header:
                f.write("ID,ImageFile,LabelFile,NumObjects\n")
                self._wrote_header = True
            f.write(f"{self.cur_sample_id},{img_path.relative_to(self.db_dir)},{label_path.relative_to(self.db_dir)},{len(yolo_lines)}\n")

        self.cur_sample_id += 1
        response.success = True
        response.message = "Collected sample successfully"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraCollector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
