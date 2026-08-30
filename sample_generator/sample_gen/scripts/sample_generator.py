#!/usr/bin/env python3

import math
import time
import random
from pathlib import Path
from typing import Any, List, Optional
from dataclasses import dataclass
import traceback

import pandas as pd
import numpy as np
import rclpy
from rclpy.client import Client
from std_srvs.srv import Empty, Trigger

from ament_index_python import get_package_share_directory

from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import MarkerArray, Marker

try:
    from arculus_common import BaseNode, BaseParams  # type: ignore
except ImportError:
    from sample_gen.base import BaseNode, BaseParams
try:
    from brain3_common import ArculeeServices  # type: ignore
except ImportError:
    from sample_gen.services import make_services_namespace
    ArculeeServices = make_services_namespace()
from sample_gen_msgs.srv import CollectSample, CheckModelsLoaded, GetCurrentSampleID
from sample_gen_msgs.msg import LoadedModelInfo

from sample_gen.box_np_ops import center_to_corner_box2d
from sample_gen.geometry import box_collision_test, points_in_2d_polygon_jit
from sample_gen.create_markers import create_box_corner_marker, create_spawn_area_marker, delete_selected_markers
from sample_gen.helpers import (
    create_ros_service_client,
    get_pose_2d_msg,
    get_pose_from_pose_2d_msg,
    load_yaml_entry,
    spawn_entity_process,
    delete_entity_process,
    set_pose_process,
)

NODE_NAME = "sample_generator_node"
SAMPLE_GEN_PKG = "sample_gen"
CONFIG_PATH = Path(get_package_share_directory(SAMPLE_GEN_PKG), "config", "config.yml")


@dataclass
class SampleGeneratorParams(BaseParams):
    database_dir: str = "/brain2/ros2/ros2_ws/install/sample_gen/share/sample_gen/database"
    visualize: bool = False
    num_batches: int = 0
    num_samples_per_batch: int = 0
    distance: float = 0.0
    max_num_tries: int = 0
    world_name: str = ""
    frame_id: str = "world"  # TODO: (james) Actually use this param throughout the file
    spawn_area_yml_name: str = ""
    spawn_area_name: str = ""
    class_config_yml: str = "class_configs.yml"
    class_config_name: str = "class_config"
    class_config_dict_name: str = "classes"  # TODO: (James) Is this necessary


class SampleGenerator(BaseNode):
    def __init__(self):
        self._params: SampleGeneratorParams = SampleGeneratorParams()  # pyright: ignore [reportIncompatibleVariableOverride]

        super().__init__(NODE_NAME)

        self.spawn_area = np.array(
            load_yaml_entry(self._params.spawn_area_name, "sample_gen", self._params.spawn_area_yml_name)
        )
        self.spawn_area_marker = create_spawn_area_marker(frame_id="world", spawn_area=list(self.spawn_area))
        self.spawn_area = np.array(self.spawn_area, dtype=float).reshape(-1, 2)

        # Get self.spawn_info - dict = {["class"] : ["num_instances"]}

        self.class_config = load_yaml_entry(
            self._params.class_config_name, SAMPLE_GEN_PKG, self._params.class_config_yml
        )

        if self.class_config:
            self.class_config_dict = self.class_config[self._params.class_config_dict_name]
            self.spawn_info = {
                x["class"]: x["num_instances"] for x in self.class_config_dict if "class" and "num_instances" in x
            }
        else:
            self.get_logger().error("Class config parameter is empty, node will not be able to check scaling")

        if self._params.visualize:
            self.corner_pub = self.create_publisher(MarkerArray, "ModelHandler/CornerMarkers", 1)
            self.spawn_area_pub = self.create_publisher(Marker, "ModelHandler/SpawnArea", 1)

        self.model_db_path = Path(self._params.database_dir, "model_db.csv")
        assert self.model_db_path.is_file(), f"Model database not found: {self.model_db_path}"
        try:
            self.model_db = pd.read_csv(self.model_db_path)
            # Check if models from db are valid
            for _, row in self.model_db.iterrows():
                model_path_str = str(row["ModelPath"])
                model_path = Path(model_path_str)
                if not model_path.is_file():
                    self.get_logger().warn(f"\n\nFound invalid model file in database: {model_path}")
                    self.get_logger().warn(f"Remove database: {self.model_db_path}")
                    self.get_logger().warn("And rerun the import_models.py script")
                    self.destroy_node()
                    return
        except Exception as e:
            self.get_logger().error(f"Could not read database: {self.model_db_path}: {e}")
            self.get_logger().error("Rerun the import_models.py script")
            self.destroy_node()
            return

        self.spawn_entity_service_name: str = f"world/{self._params.world_name}/create"
        self.delete_entity_service_name: str = f"world/{self._params.world_name}/remove"
        self.set_pose_service_name: str = f"world/{self._params.world_name}/set_pose"

        if self._params.visualize:
            self.num_markers_before = 0
            self.corner_pub = self.create_publisher(MarkerArray, "ModelHandler/CornerMarkers", 1)
            self.spawn_area_pub = self.create_publisher(Marker, "ModelHandler/SpawnArea", 1)
        else:
            self.corner_pub = None
            self.spawn_area_pub = None

        self.collect_sample_client: Client = create_ros_service_client(
            ArculeeServices.COLLECT_SAMPLE, CollectSample, self
        )
        self.unregister_gt_subs_client: Client = create_ros_service_client(
            ArculeeServices.UNREGISTER_GT_SUBS, Empty, self
        )
        self.update_gt_subs_client: Client = create_ros_service_client(ArculeeServices.UPDATE_GT_SUBS, Trigger, self)
        self.check_models_loaded_client: Client = create_ros_service_client(
            ArculeeServices.CHECK_MODELS_LOADED, CheckModelsLoaded, self
        )
        self.get_current_sample_id_client = create_ros_service_client(
            ArculeeServices.GET_CURRENT_SAMPLE_ID, GetCurrentSampleID, self
        )
        self.get_logger().info("Initialized")

    # def _adjust_plugin_name(self, path, name):
    #     # Get xml tree
    #     tree: ET.ElementTree = ET.parse(path)
    #     root = tree.getroot()
    #     model = root.find("model")
    #     if model is not None:
    #         plugin = model.find("plugin")
    #     if plugin is not None:
    #         plugin.attrib["name"] = f"{name}_label_plugin"
    #     tree.write(path)

    def _spawn_models(self, num_batch):
        loaded_boxes = []
        current_id = 0

        # Pick this batch's total tree count (uniform 1..10, so the dataset
        # spans the full range from a single tree up to a dense ~10-tree forest),
        # then choose the composition: ~60% mixed, ~20% all of one class,
        # ~20% all of the other.
        classes = list(self.spawn_info.keys())
        max_total = min(sum(self.spawn_info.values()), 10)
        total = random.randint(1, max_total)

        counts = {c: 0 for c in classes}
        if len(classes) >= 2:
            # Deterministic composition per batch (num_batch % 5) instead of a
            # random roll, so the dataset is 60% mixed / 20% all-pine /
            # 20% all-oak AND resuming a run reproduces the same composition
            # for each batch index.
            batch_type = num_batch % 5  # 0,1,2 = mixed, 3 = all last class, 4 = all first class
            if batch_type == 3:
                counts[classes[-1]] = total
            elif batch_type == 4:
                counts[classes[0]] = total
            elif total > 1:
                split = random.randint(1, total - 1)
                counts[classes[0]] = split
                counts[classes[-1]] = total - split
            else:
                counts[random.choice(classes)] = total
        else:
            counts[classes[0]] = total
        num_models_total = sum(counts.values())

        if self._params.visualize and self.spawn_area_pub is not None:
            self.spawn_area_pub.publish(self.spawn_area_marker)

        model_info_list = []
        for classification, num_instances in self.spawn_info.items():
            current_db = self.model_db.loc[self.model_db["Type"] == classification]
            # Skip if no models with class in database
            if len(current_db) == 0:
                continue
            current_db.reset_index(inplace=True)
            for _ in range(counts.get(classification, num_instances)):
                model_idx = np.random.randint(len(current_db.index))
                model_name = current_db.loc[model_idx]["Name"] + f"_{num_batch}_{current_id}"

                file_dir = current_db.loc[model_idx]["ModelPath"]
                length = current_db.loc[model_idx]["BboxLength"]
                width = current_db.loc[model_idx]["BboxWidth"]
                offsets = [current_db.loc[model_idx]["OffsetX"], current_db.loc[model_idx]["OffsetY"]]
                (spawn_pose, new_box) = self._get_random_pose(
                    length, width, inside_area=True, boxes_offline=loaded_boxes, box_offsets=offsets
                )
                if spawn_pose is None:
                    continue

                # self._adjust_plugin_name(file_dir, model_name) #TODO: (james) Plugin must match namespace?

                self.get_logger().info(f"Spawn pose: {spawn_pose}")

                if (
                    spawn_entity_process(
                        self, self._params.world_name, model_name, file_dir, get_pose_from_pose_2d_msg(spawn_pose)
                    )
                    is None
                ):
                    self.get_logger().info("spawn_entity_process returned none...")
                    return None

                current_id += 1
                model_info = {}
                model_info["name"] = model_name
                model_info["pos_x"] = spawn_pose.x
                model_info["pos_y"] = spawn_pose.y
                model_info["offset_x"] = offsets[0]
                model_info["offset_y"] = offsets[1]
                model_info["pose_2d"] = spawn_pose
                model_info["bbox_length"] = length
                model_info["bbox_width"] = width
                model_info["yaw"] = spawn_pose.theta
                model_info_list.append(model_info)
                loaded_boxes.append(new_box)

                self.get_logger().info(f"Loading model: {current_id} of {num_models_total}")

        return model_info_list

    def _delete_models(self, model_info_list: List[Any]) -> bool:
        model_names: list[str] = [x["name"] for x in model_info_list]
        for name in model_names:
            if not delete_entity_process(self, self._params.world_name, name):
                self.get_logger().error(f"Error deleting model {name}")
                return False
            time.sleep(1.0)
        return True

    def _move_models(self, model_info_list: List[Any]) -> Optional[Any]:
        if self._params.visualize and self.spawn_area_pub:
            self.spawn_area_pub.publish(self.spawn_area_marker)

        model_names: list[str] = [x["name"] for x in model_info_list]
        model_boxes: list[list[float]] = [
            [x["pos_x"], x["pos_y"], x["bbox_length"], x["bbox_width"], x["yaw"]] for x in model_info_list
        ]
        model_offsets = [[x["offset_x"], x["offset_y"]] for x in model_info_list]

        # Place models to random position in spawn_area (trees are static and
        # were already spawned inside the area, so no "move far away first"
        # pass is needed - and skipping it avoids set_pose reliability issues).
        for i, name in enumerate(model_names):
            boxes_offline = model_boxes[:]
            model_box = boxes_offline.pop(i)
            new_box = model_box[:]
            (pose, new_box_rand) = self._get_random_pose(
                new_box[2], new_box[3], inside_area=True, boxes_offline=boxes_offline, box_offsets=model_offsets[i]
            )

            # Store new box and move model to new position if pose found
            if pose is not None:
                if new_box_rand:
                    model_boxes[i] = new_box_rand

                if not set_pose_process(self, get_pose_from_pose_2d_msg(pose), self._params.world_name, name):
                    return None

        model_info_list = []
        for i, name in enumerate(model_names):
            model_info = {}
            model_info["name"] = name
            model_info["pos_x"] = model_boxes[i][0]
            model_info["pos_y"] = model_boxes[i][1]
            model_info["bbox_length"] = model_boxes[i][2]
            model_info["bbox_width"] = model_boxes[i][3]
            model_info["offset_x"] = model_offsets[i][0]
            model_info["offset_y"] = model_offsets[i][1]
            model_info["yaw"] = model_boxes[i][4]
            model_info_list.append(model_info)

        return model_info_list

    def _check_collisions(self, box_new_model, boxes_offline=[]):
        # Already in world frame
        new_box = np.array(box_new_model).reshape(-1, 5)
        new_box[:, 2:4] += self._params.distance
        new_box_corners = center_to_corner_box2d(new_box[:, 0:2], new_box[:, 2:4], new_box[:, 4])

        boxes_2d = []
        for box in boxes_offline:
            # Map frame already
            box2D = np.array(box).reshape(5)  # x, y, l, w, yaw
            boxes_2d.append(box2D)

        if len(boxes_2d) > 0 and self._params.distance:
            boxes_2d = np.array(boxes_2d).reshape(-1, 5)
            boxes_2d[:, 2:4] += self._params.distance
            box_corners = center_to_corner_box2d(boxes_2d[:, 0:2], boxes_2d[:, 2:4], boxes_2d[:, 4])
        else:
            return False  # No collisions if no boxes

        if self._params.visualize:
            corner_markers = MarkerArray()
            corner_markers.markers = []

            # Create marker for new box
            create_box_corner_marker(
                box_corners=new_box_corners[0, :, :],
                frame_id="world",
                marker_array=corner_markers,
                colour=[1.0, 0.0, 0.0],
            )

            num_markers = box_corners.shape[0]

            # Delete excess markers
            if num_markers < self.num_markers_before:
                ids_to_delete = list(range(num_markers, self.num_markers_before))
                if self.corner_pub:
                    delete_selected_markers(self.corner_pub, ids_to_delete)

            # Publish corners of other boxes
            for j in range(num_markers):
                create_box_corner_marker(
                    box_corners=box_corners[j, :, :],
                    frame_id="world",
                    marker_array=corner_markers,
                    colour=[0.0, 1.0, 0.0],
                )
            if self.corner_pub:
                self.corner_pub.publish(corner_markers)
            time.sleep(0.4)

        self.num_markers_before = box_corners.shape[0]

        collisions = box_collision_test(box_corners, new_box_corners)
        return np.any(collisions)

    def _transform_offsets(self, offsets, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        offset_x = sin_yaw * offsets[1] + cos_yaw * offsets[0]
        offset_y = cos_yaw * offsets[1] - sin_yaw * offsets[0]
        return offset_x, offset_y

    def _get_random_pose_in_polygon(self, length, width, box_offsets):
        in_polygon = False
        area_max_x = np.max(self.spawn_area[:, 0])
        area_min_x = np.min(self.spawn_area[:, 0])
        area_max_y = np.max(self.spawn_area[:, 1])
        area_min_y = np.min(self.spawn_area[:, 1])
        while not in_polygon:
            # Bias x toward the near edge (u^2) so the dataset includes close-up
            # trees - a uniform sample would mostly land far away, where the FOV
            # cone is wide and the trapezoid has most of its area.
            u = random.random()
            position_x: float = area_min_x + (area_max_x - area_min_x) * (u * u)
            position_y: float = random.uniform(area_max_y, area_min_y)
            yaw: float = random.uniform(-math.pi, math.pi)
            offset_x, offset_y = self._transform_offsets(box_offsets, -yaw)

            new_box = [position_x + offset_x, position_y + offset_y, length, width, yaw]  # needs to be a list?
            new_box = np.array(new_box).reshape(-1, 5)  # x, y, l, w, yaw
            cornerPoints = center_to_corner_box2d(centers=new_box[:, 0:2], dims=new_box[:, 2:4], angles=new_box[:, 4])
            cornerPoints = cornerPoints.reshape(-1, 2)

            result = points_in_2d_polygon_jit(cornerPoints, self.spawn_area)
            in_polygon = np.all(result)

            if self._params.visualize:
                corner_markers = MarkerArray()
                corner_markers.markers = []
                if in_polygon:
                    colour = [0.0, 1.0, 0.0]
                else:
                    colour = [1.0, 0.0, 0.0]
                create_box_corner_marker(
                    box_corners=cornerPoints,
                    frame_id="world",
                    ns="rand_pose_polygon",
                    marker_array=corner_markers,
                    colour=colour,
                    lifetime=False,
                )
                if self.corner_pub:
                    self.corner_pub.publish(corner_markers)
                time.sleep(0.4)

        return (float(position_x) + float(offset_x), float(position_y) + float(offset_y), offset_x, offset_y, yaw)

    def _get_random_pose(
        self,
        length: float,
        width: float,
        inside_area: float,
        boxes_offline: List[Any] = [],
        box_offsets: List[float] = [0.0, 0.0],
    ) -> tuple[Optional[Pose2D], Optional[list[float]]]:
        pose_occupied = True
        num_tries = 0
        while pose_occupied:
            if inside_area:
                if num_tries > self._params.max_num_tries:
                    return (None, None)

            if inside_area:
                (position_x, position_y, offset_x, offset_y, yaw) = self._get_random_pose_in_polygon(
                    length, width, box_offsets
                )
            else:
                # Make spawn area large and far away
                position_x = random.uniform(-20000, -25000) + box_offsets[0]
                position_y = random.uniform(-20000, 20000) + box_offsets[1]
                offset_x = box_offsets[0]
                offset_y = box_offsets[1]
                yaw = 0.0

            new_box = [position_x, position_y, length, width, yaw]
            pose_occupied = self._check_collisions(new_box, boxes_offline)
            num_tries += 1

        return (
            get_pose_2d_msg(x=float(position_x) - float(offset_x), y=float(position_y) - float(offset_y), yaw=yaw),
            new_box,
        )

    def run_sample_generation(self) -> None:
        # Wrap in try - except to avoid gazebo and rviz from keep running if error occurs
        try:
            # Initial update of subscribers
            result: Trigger.Response = self._call_service(self.update_gt_subs_client, Trigger.Request())
            if result.success:
                for i in range(self._params.num_batches):
                    # Skip batches already completed in a previous (crashed) run.
                    if self._get_current_sample_id() >= self._params.num_samples_per_batch * (i + 1):
                        self.get_logger().info(f"Batch {i + 1} already complete, skipping")
                        continue
                    model_info_list = self._spawn_models(i)
                    if model_info_list is None:
                        self.get_logger().error("Error when spawning model: model_info_list is None")
                        return

                    # # Update gt_subs first and then wait until all models loaded
                    self._call_service(self.update_gt_subs_client, Trigger.Request())

                    while not self._check_models_loaded_request(len(model_info_list)):
                        self.get_logger().info("Sending service request to SampleCollector/update_gt_subs")
                        self._call_service(self.update_gt_subs_client, Trigger.Request())
                        time.sleep(0.2)

                    self.get_logger().info(f"\n==== Loaded models succesfully, start with batch {i + 1} ====")

                    # Loop for creating samples
                    while self._get_current_sample_id() < self._params.num_samples_per_batch * (i + 1):
                        model_info_list = self._move_models(model_info_list)
                        if model_info_list is None:
                            self.get_logger().error("Error when moving models")
                            return

                        self._collect_sample(model_info_list)

                        first = self._get_current_sample_id() - (i * self._params.num_samples_per_batch)
                        current = first / self._params.num_samples_per_batch * 100
                        indicator = int(current / 10)
                        self.get_logger().info(
                            f"[{'=' * indicator:<10}] {current:.0f}% ({first} of {self._params.num_samples_per_batch}) "
                            f"(Batch {i + 1} of {self._params.num_batches})"
                        )

                    self._unregister_gt_subs()
                    time.sleep(1.0)

                    if not self._delete_models(model_info_list):
                        return
                    time.sleep(1.0)

                    # self._call_service(self.update_gt_subs_client, Trigger.Request())
                    # # Wait until all models deleted
                    # while not self._check_models_loaded_request(0):
                    #     self.get_logger().debug("Sending service request to SampleCollector/update_gt_subs")
                    #     self._call_service(self.update_gt_subs_client, Trigger.Request())
                    #     time.sleep(0.01)

                self.get_logger().info(
                    f"\n\nFinished creating samples. {self._get_current_sample_id()} samples in total\n\n"
                )

            else:
                self.get_logger().warn(
                    "Could not update groundtruth subscribers, are models already loaded and no data available?"
                )
        except Exception:
            self.get_logger().error(traceback.format_exc())
            self.get_logger().error("\n\nError occured, shutting everything down")

    def _call_service(self, client: Client, request: Any) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _check_models_loaded_request(self, num_models: int) -> Any:
        request = CheckModelsLoaded.Request()
        request.num_models = num_models
        response = self._call_service(self.check_models_loaded_client, request)
        return response.success

    def _get_current_sample_id(self) -> Any:
        response = self._call_service(self.get_current_sample_id_client, GetCurrentSampleID.Request())
        return response.sample_id

    def _collect_sample(self, model_info_list: List[Any]) -> None:
        self.get_logger().info("Run _collect_sample")
        success = False
        request = CollectSample.Request()
        request.model_info = []
        for info in model_info_list:
            model_info = LoadedModelInfo()
            model_info.name = info["name"]
            model_info.bbox_length = info["bbox_length"]
            model_info.bbox_width = info["bbox_width"]
            model_info.pose_2d.x = info["pos_x"]
            model_info.pose_2d.y = info["pos_y"]
            model_info.pose_2d.theta = info["yaw"]
            model_info.offset_x = info["offset_x"]
            model_info.offset_y = info["offset_y"]
            request.model_info.append(model_info)
        # Stamp the request ONCE with the moment the models finished moving. The
        # collector then waits until a camera frame captured AFTER this time is
        # available, so the image always matches the current tree positions
        # (previously a fresh-enough image could show the trees' old positions).
        # Keep the stamp fixed across retries - advancing it would re-stale the
        # frame each loop.
        request.stamp = self.get_clock().now().to_msg()
        while not success:
            response = self._call_service(self.collect_sample_client, request)
            success = response.success
            self.get_logger().info("Run _collect_sample not successful, wait 0.1 and try again")
            time.sleep(0.2)

    def _unregister_gt_subs(self) -> None:
        self._call_service(self.unregister_gt_subs_client, Empty.Request())


def main(args=None):
    rclpy.init(args=args)

    node = SampleGenerator()
    node.run_sample_generation()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
