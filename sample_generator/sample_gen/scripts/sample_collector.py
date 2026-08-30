#!/usr/bin/env python3
import numpy as np
import time
import copy
from functools import partial
from glob import glob
from typing import Any, List, Optional, Tuple
from pathlib import Path
from dataclasses import dataclass, field


from ament_index_python import get_package_share_directory
try:
    from arculus_common import BaseNode, BaseParams  # type: ignore
except ImportError:
    from sample_gen.base import BaseNode, BaseParams
try:
    from brain3_common import ArculeeTopics, ArculeeServices  # type: ignore
except ImportError:
    from sample_gen.services import make_services_namespace, make_topics_namespace
    ArculeeServices = make_services_namespace()
    ArculeeTopics = make_topics_namespace()
import rclpy
import rclpy.parameter
import rclpy.qos
from rclpy.time import Time as ROSTime
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Empty, Trigger
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import ros2_numpy as rnp

from sample_gen_msgs.msg import ObjectLabel
from sample_gen_msgs.srv import CollectSample, CheckModelsLoaded, GetCurrentSampleID, GetDBData
from sample_gen.helpers import (
    get_pose_msg,
    load_yaml_entry,
    store_sample_with_entry,
    publish_gt_pc,
)
from sample_gen.geometry import transform_points
from sample_gen.filtering import remove_occluded_objects
from sample_gen.bbox_3d import BoundingBox3D

NODE_NAME = "sample_collector_node"
SAMPLE_GEN_PKG = "sample_gen"
CONFIG_PATH = Path(get_package_share_directory(SAMPLE_GEN_PKG), "config", "config.yml")


@dataclass
class SampleCollectorParams(BaseParams):
    database_dir: str = "/brain2/ros2/ros2_ws/install/sample_gen/share/sample_gen/database"
    visualize: bool = False
    min_points_sample: int = 0
    min_points_object: int = 0
    max_occlusion: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    frame_id: str = "arculee/base_link/laser_something_sensor "
    lidar_topics: List[str] = field(default_factory=lambda: [""])
    model_distance: float = 0.0
    class_config_yml: str = "class_configs.yml"
    class_config_name: str = "class_config"
    class_config_dict_name: str = "classes"


class SampleCollector(BaseNode):
    def __init__(self):
        self._params: SampleCollectorParams = SampleCollectorParams()  # pyright: ignore [reportIncompatibleVariableOverride]
        super().__init__(NODE_NAME)

        # Convert list -> np.ndarray
        if not isinstance(self._params.max_occlusion, np.ndarray):
            self._params.max_occlusion = np.array(self._params.max_occlusion).tolist()

        self.class_config = load_yaml_entry(
            self._params.class_config_name, SAMPLE_GEN_PKG, self._params.class_config_yml
        )

        if self.class_config:
            self.class_config_dict = self.class_config[self._params.class_config_dict_name]
            self.class_type_dict = {x["id"]: x["class"] for x in self.class_config_dict if "id" and "class" in x}
        else:
            self.get_logger().error("Class config parameter is empty, node will not be able to check scaling")

        if self._params.max_occlusion is None or len(self._params.max_occlusion) != 3:
            self.get_logger().error("Provided ROS parameter max_occlusion is invalid. Needs to have length of 3")
            self.destroy_node()
            return
        else:
            self._params.max_occlusion = np.array(self._params.max_occlusion).tolist()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gt_subs = []
        self.pc_subs = []
        self.pointclouds = []
        self.obj_labels = []
        self._update_pc_subs()

        self.cur_sample_id = 0

        # Create main dir if required
        self.db_dir = Path(self._params.database_dir)
        if not self.db_dir.exists:
            Path(self.db_dir).mkdir(parents=True, exist_ok=True)

        # # Create sub_dir
        new_db_idx = len(glob(str(self.db_dir / "**")))
        self.db_dir = Path(self.db_dir, f"generated_samples_{new_db_idx}")
        Path(self.db_dir, "points").mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Created folder at: {Path(self.db_dir, 'points')}")

        Path(self.db_dir, "groundtruth").mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Created folder at {Path(self.db_dir, 'groundtruth')}")
        self.db_path = Path(self.db_dir, "sample_database.csv")
        self.get_logger().info(f"Database path: {self.db_path}")

        if self._params.visualize:
            self.gt_orig_pub = self.create_publisher(MarkerArray, ArculeeTopics.SAMPLE_COLLECTOR_ORIGINAL_GT.value, 1)
            self.gt_clean_pub = self.create_publisher(MarkerArray, ArculeeTopics.SAMPLE_COLLECTOR_CLEANED_GT.value, 1)
            self.pc_orig_pub = self.create_publisher(
                PointCloud2, ArculeeTopics.SAMPLE_COLLECTOR_ORIGINAL_POINTS.value, 1
            )
            self.pc_clean_pub = self.create_publisher(
                PointCloud2, ArculeeTopics.SAMPLE_COLLECTOR_CLEANED_POINTS.value, 1
            )
        else:
            self.gt_orig_pub = None
            self.gt_clean_pub = None
            self.pc_orig_pub = None
            self.pc_clean_pub = None

        try:
            self.collect_sample_srv = self.create_service(
                CollectSample, ArculeeServices.COLLECT_SAMPLE.value, self._collect_sample_cb
            )

            self.unregisters_gt_subs_srv = self.create_service(
                Empty, ArculeeServices.UNREGISTER_GT_SUBS.value, self._unregister_gt_subs_cb
            )

            self.update_gt_subs_srv = self.create_service(
                Trigger, ArculeeServices.UPDATE_GT_SUBS.value, self._update_gt_subs_cb
            )
            self.get_logger().info(f"Created service for {ArculeeServices.UPDATE_GT_SUBS.value}")

            self.check_models_loaded_srv = self.create_service(
                CheckModelsLoaded, ArculeeServices.CHECK_MODELS_LOADED.value, self._check_models_loaded_cb
            )
            self.get_logger().info("Created service for check_models_loaded")

            self.get_current_sample_id_srv = self.create_service(
                GetCurrentSampleID, ArculeeServices.GET_CURRENT_SAMPLE_ID.value, self._get_current_sample_id_cb
            )

            self.get_gt_data_srv = self.create_service(
                GetDBData, ArculeeServices.GET_GT_DATA.value, self._get_gt_data_cb
            )

        except Exception as e:
            self.get_logger().info(f"Error: {e}")

        self.get_logger().info("Initialized")

    def _object_label_cb(self, idx: int, msg: ObjectLabel) -> None:
        # self.get_logger().info('Received groundtruth data')
        self.obj_labels[idx][0] = msg
        # self.get_logger().info(f"self.object_labels: {self.obj_labels}")

    def _pointcloud_cb(self, idx: int, msg: PointCloud2) -> None:
        # self.get_logger().info('Received pointcloud data')
        self.pointclouds[idx] = msg

    def _get_current_sample_id_cb(
        self, request: GetCurrentSampleID.Request, response: GetCurrentSampleID.Response
    ) -> GetCurrentSampleID.Response:
        response.sample_id = self.cur_sample_id
        return response

    def _get_gt_data_cb(self, request: Any, response: Any) -> Any:
        self.get_logger().info("Getting GT Data")
        self.get_logger().info("Checking for new data")
        response = self._check_new_data(ROSTime.from_msg(request.stamp), response, withPC=False)
        if not response.success:
            self.get_logger().info("No new data found")
            return response

        self.get_logger().info("Getting groundtruth")
        groundtruth = self._get_gt(ROSTime.from_msg(request.stamp))
        if groundtruth is None:
            response.message = "Issue when transforming groundtruth"
            response.success = False
            self.get_logger().info(f"{response.message}")
            return response
        else:
            bboxes, classifications, types, _ = groundtruth
        offsets = self._get_offsets()

        response.success = True
        response.size.x = float(bboxes[0, 3])
        response.size.y = float(bboxes[0, 4])
        response.size.z = float(bboxes[0, 5])
        response.offset.x = float(offsets[0, 0])
        response.offset.y = float(offsets[0, 1])
        response.offset.z = float(offsets[0, 2])
        response.classification = int(classifications[0])
        response.class_name = types[0]
        return response

    def _unregister_gt_subs(self) -> None:
        if self.gt_subs:
            for sub in self.gt_subs:
                self.destroy_subscription(sub)
            self.gt_subs.clear()

    def _check_new_data_list(self, data_list: List, stamp: ROSTime) -> Optional[bool]:
        if data_list:
            # Check stamps of all states
            for data in data_list:
                if data is None:
                    self.get_logger().info(f"Data {data} was found to be None in datalist {data_list}")
                    return None
                dt = stamp - ROSTime.from_msg(data.header.stamp)
                if dt.nanoseconds > 0:
                    self.get_logger().info(f"dt {dt} > 0")
                    return False
            return True

    def _check_data_list(self, data_list: List) -> bool:
        if len(data_list) > 0:
            for data in data_list:
                if data is None:
                    return False
        return True

    def _update_pc_subs(self) -> None:
        self.get_logger().info("Updating point cloud subs...")
        # Delete old subs
        if self.pc_subs:
            for sub in self.pc_subs:
                self.destroy_subscription(sub)
            self.pointclouds.clear()
            self.pc_subs.clear()

        self.get_logger().info("Getting lidar point cloud topics now")
        # Initialize subscribers for all pc topics
        if self._params.lidar_topics is not None:
            if len(self._params.lidar_topics) > 0:
                self.pointclouds = []
                for i, topic in enumerate(self._params.lidar_topics):
                    self.pointclouds.append(None)
                    cb_func = partial(self._pointcloud_cb, i)

                    new_sub = self.create_subscription(
                        PointCloud2, topic, cb_func, qos_profile=rclpy.qos.qos_profile_sensor_data
                    )
                    self.get_logger().info(f"Created pointcloud subscription to {topic}")
                    self.pc_subs.append(new_sub)
        else:
            self.get_logger().warn("No lidar topics specified")

    def _init_object_label_subs(self):
        try:
            topics = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f"Couldn't get published topics: {e}")
            return

        self.obj_labels.clear()
        self.gt_subs.clear()
        idx = 0

        for topicName, topicType in topics:
            # Only read ObjectLabel message types
            if topicType[0] == "sample_gen_msgs/msg/ObjectLabel":
                # rospy.loginfo("RosSampleHandler: Found ObjectLabel of model: %s", topic[0])
                cb_func = partial(self._object_label_cb, idx)
                new_gt_sub = self.create_subscription(ObjectLabel, topicName, cb_func, 1)
                self.gt_subs.append(new_gt_sub)
                self.obj_labels.append([None, topicName.split("/")[2]])
                idx += 1

    def _check_new_data(
        self, stamp: ROSTime, response: CollectSample.Response, withPC: bool = True
    ) -> CollectSample.Response:
        if withPC:
            resultPC = self._check_new_data_list(self.pointclouds, stamp)
            self.get_logger().info(f"resultPC is: {resultPC}")
            if resultPC is None:
                response.success = False
                response.message = "ERROR, PC TOPIC NOT INITIALIZED"
                return response
            if resultPC is False:
                response.success = False
                response.message = "PC not up to date"
                return response
        else:
            resultPC = True

        resultGT = self._check_new_data_list([x[0] for x in self.obj_labels], stamp)
        if resultGT is None:
            response.success = False
            response.message = "ERROR, STATE TOPIC NOT INITIALIZED"
            return response
        if resultGT is False:
            response.success = False
            response.message = "Groundtruth not up to date"
            return response

        response.success = resultPC and resultGT
        return response

    def _update_gt_subs_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._update_gt_subs()
        response.success = True
        response.message = ""
        return response

    def _update_gt_subs(self) -> None:
        self._unregister_gt_subs()
        self._init_object_label_subs()

    def _unregister_gt_subs_cb(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self._unregister_gt_subs()
        return response

    def _check_models_loaded_cb(
        self, request: CheckModelsLoaded.Request, response: CheckModelsLoaded.Response
    ) -> CheckModelsLoaded.Response:
        groundtruth = self._get_gt()
        # self.get_logger().info(f"Groundtruths: {groundtruth}")
        if groundtruth is None or len(groundtruth[0]) == 0:
            response.message = "Issue when transforming groundtruth"
            response.success = False
            self.get_logger().info(f"No groundtruths found: {response.message}")
            return response
        else:
            bboxes, _, _, _ = groundtruth
            self.get_logger().info(f"Found groundtruths: {groundtruth}")

        if len(bboxes) == request.num_models:
            response.success = True
        else:
            response.success = False
            message = f"Found {len(bboxes)} models, expected: {request.num_models}"
            self.get_logger().info(message)
            response.message = "Could not find groundtruth of all models"
        return response

    def _get_pc(self, stamp: ROSTime) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        # Deserialize pointcloud
        # Copy to prevent from being overwritten
        copies = [copy.deepcopy(x) for x in self.pointclouds]
        combined_points: np.ndarray = np.empty((0, 3), dtype=np.float32)
        front_points: np.ndarray = np.empty((0, 3), dtype=np.float32)
        rear_points: np.ndarray = np.empty((0, 3), dtype=np.float32)

        for i, cpy in enumerate(copies):
            trafo = self._get_trafo(cpy.header.frame_id, stamp)
            if trafo is None:
                return None
            pc = rnp.numpify(cpy)
            if pc is not None:
                points = np.zeros((pc.shape[0], 3), dtype=np.float32)
                points[:, 0] = pc["x"]
                points[:, 1] = pc["y"]
                points[:, 2] = pc["z"]
                points_trafo = transform_points(points, trafo)
            if i == 0:
                front_points = points_trafo
            elif i == 1:
                rear_points = points_trafo
            if len(combined_points) > 0:
                combined_points = np.concatenate((combined_points, points_trafo), axis=0)
            else:
                combined_points = points_trafo

        return combined_points, front_points, rear_points

    def _get_gt(
        self, stamp: Optional[ROSTime] = None
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        bboxes = []
        classifications = []
        names = []
        types = []

        for label_gt, label_name in self.obj_labels:
            if label_gt:
                if stamp:
                    pose = self._transform_pose(label_gt, stamp)
                    self.get_logger().info(f"self.transformed_pose: {pose}")
                    if pose is None:
                        return None
                else:
                    pose = get_pose_msg()
                bbox3D = BoundingBox3D.from_ros_msgs(label_gt, pose, self.class_type_dict)
                # self.get_logger().info(f"Full bbox3D: {bbox3D.box3d}")

                bboxes.append(bbox3D.box3d)
                classifications.append(bbox3D.classification)
                names.append(label_name)
                types.append(bbox3D.type)

            else:
                continue

        bboxes = np.array(bboxes).reshape(-1, 7)
        classifications = np.array(classifications)
        names = np.array(names)
        types = np.array(types)

        return (bboxes, classifications, types, names)

    def _get_offsets(self) -> np.ndarray:
        offsets = []
        for label_gt, _ in self.obj_labels:
            offsets.append([label_gt.offset_x, label_gt.offset_y, label_gt.offset_z])

        return np.array(offsets).reshape(-1, 3)

    def _transform_pose(self, label: ObjectLabel, stamp: ROSTime) -> Optional[Pose]:
        # return pose if both frames are equal
        if label.header.frame_id.replace("/", "") == self._params.frame_id.replace("/", ""):
            return label.pose
        trafo = self._get_trafo(label.header.frame_id, stamp)
        if trafo is None:
            return None
        pos = label.pose.position
        position = np.array([pos.x, pos.y, pos.z], dtype="float32").reshape(-1, 3)
        trafo_position = transform_points(position, trafo)
        pose_trafo = Pose()
        pose_trafo.position.x = float(trafo_position[0, 0])
        pose_trafo.position.y = float(trafo_position[0, 1])
        pose_trafo.position.z = float(trafo_position[0, 2])
        pose_trafo.orientation = trafo.transform.rotation

        return pose_trafo

    def _get_trafo(self, source_frame: str, stamp: ROSTime) -> Optional[TransformStamped]:
        try:
            trafo = self.tf_buffer.lookup_transform(self._params.frame_id, source_frame, stamp)
            self.get_logger().info(
                f"Was able to look up transforms from {self._params.frame_id} to {source_frame}. Resulting transform returned: {trafo}"
            )
            return trafo
        except Exception as ex:
            self.get_logger().warn(f"Could not transform {self._params.frame_id} to {source_frame}: {ex}")
            return None

    def _collect_sample_cb(
        self, request: CollectSample.Request, response: CollectSample.Response
    ) -> CollectSample.Response:
        stamp: ROSTime = ROSTime.from_msg(request.stamp)
        self.get_logger().info("Collecting sample, checking for new data first")
        response = self._check_new_data(stamp, response)
        if not response.success:
            self.get_logger().info(f"Data check not successful: {response.message}")
            return response

        # Get newest stamp of all pointclouds to make sure the groundtruth actually matches!
        points = self._get_pc(stamp)
        if points is None:
            response.message = "Issue when transforming point cloud"
            response.success = False
            self.get_logger().info(f"{response.message}")
            return response

        combined_points, front_points, rear_points = points

        groundtruth = self._get_gt(stamp)
        if groundtruth is None:
            response.message = "Issue when transforming groundtruth"
            response.success = False
            self.get_logger().info(f"{response.message}")
            return response
        else:
            bboxes, _, types, names = groundtruth

        # Check expected positions of groundtruths
        for i in range(bboxes.shape[0]):
            x = bboxes[i, 0]
            y = bboxes[i, 1]
            name = str(names[i])
            found_match = False
            self.get_logger().info(f"Check gts for {name}")
            for info in request.model_info:
                info.name = info.name.replace(
                    "-", "_"
                )  # Due to issue with plugin not being able to publish onto topics with '-'

                if info.name != name:
                    continue

                diff_pos = [abs(x - info.pose_2d.x), abs(y - info.pose_2d.y)]
                if all(diffs < 5.0 for diffs in diff_pos):
                    found_match = True
                break
            if not found_match:
                response.message = "Could not find expected groundtruth, wait until another update of groundtruths"
                response.success = False
                self.get_logger().info(f"{response.message}")
                return response

        if self._params.visualize:
            publish_gt_pc(
                self.get_clock().now().to_msg(),
                points=combined_points,
                groundtruth=bboxes,
                pc_pub=self.pc_orig_pub,
                gt_pub=self.gt_orig_pub,
                colour=[1.0, 0.0, 0.0],
                frame_id=self._params.frame_id,
            )

        # Remove occluded objects from pc and gt
        remove_points_mask_combined, keep_boxes_mask, occlusions = remove_occluded_objects(
            combined_points,
            bboxes,
            max_occlusion=np.array(self._params.max_occlusion),
            min_points_object=self._params.min_points_object,
        )
        cleaned_combined_points = combined_points[~remove_points_mask_combined, :]
        cleaned_gt = bboxes[keep_boxes_mask, :]
        cleaned_types = types[keep_boxes_mask]
        cleaned_names = names[keep_boxes_mask]

        # Remove occluded objects from front and rear point clouds
        remove_points_mask_front, _, _ = remove_occluded_objects(
            front_points,
            bboxes,
            max_occlusion=np.array(self._params.max_occlusion),
            min_points_object=self._params.min_points_object,
        )
        cleaned_front_points = front_points[~remove_points_mask_front, :]

        remove_points_mask_rear, _, _ = remove_occluded_objects(
            rear_points,
            bboxes,
            max_occlusion=np.array(self._params.max_occlusion),
            min_points_object=self._params.min_points_object,
        )
        cleaned_rear_points = rear_points[~remove_points_mask_rear, :]

        if cleaned_combined_points.shape[0] < self._params.min_points_sample:
            response.success = False
            response.message = f"Only {cleaned_combined_points.shape[0]} points in sample, minimum required is: {self._params.min_points_sample}"
            self.get_logger().info(f"{response.message}")
            return response

        if self._params.visualize:
            publish_gt_pc(
                self.get_clock().now().to_msg(),
                points=cleaned_combined_points,
                groundtruth=cleaned_gt,
                pc_pub=self.pc_clean_pub,
                gt_pub=self.gt_clean_pub,
                colour=[0.0, 1.0, 0.0],
                frame_id=self._params.frame_id,
                with_line=True,
            )
            time.sleep(0.5)

        cleaned_points_dict = {
            "combined": cleaned_combined_points,
            "front": cleaned_front_points,
            "rear": cleaned_rear_points,
        }

        store_sample_with_entry(
            cleaned_points_dict,
            cleaned_gt,
            cleaned_types,
            cleaned_names,
            occlusions,
            self.cur_sample_id,
            self.db_dir,
            self.db_path,
        )
        self.cur_sample_id += 1
        response.success = True
        self.get_logger().info("Collected sample successfully")
        return response


def main(args=None):
    rclpy.init(args=args)

    node = SampleCollector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
