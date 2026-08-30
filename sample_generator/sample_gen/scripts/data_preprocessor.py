#!/usr/bin/env python3

import numpy as np
from typing import Any, List, Tuple
from pathlib import Path
from dataclasses import dataclass, field
import ast

import pandas as pd
import rclpy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2

try:
    from arculus_common import BaseNode, BaseParams  # type: ignore
except ImportError:
    from sample_gen.base import BaseNode, BaseParams
try:
    from brain3_common.name_defs import ArculeeTopics  # type: ignore
except ImportError:
    from sample_gen.services import make_topics_namespace
    ArculeeTopics = make_topics_namespace()
from sample_gen.box_np_ops import center_to_corner_box2d
from sample_gen.geometry import box_collision_test
from sample_gen.helpers import (
    store_sample_with_entry,
    publish_gt_pc,
    progress_bar_update,
)
from sample_gen.filtering import remove_occluded_objects
from sample_gen.bbox_3d import BoundingBox3D

NODE_NAME = "data_preprocessor_node"
SAMPLE_GEN_PKG = "sample_gen"


@dataclass
class DataPreprocessorParams(BaseParams):
    database_dir: str = "/brain2/ros2/ros2_ws/install/sample_gen/share/sample_gen/database"
    min_points_sample: int = 0
    min_number_objects: int = 0
    min_points_object: int = 0  # Is this needed? Not used in this node anywhere
    max_occlusion: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    frame_id: str = ""
    training_split: float = 0.0
    visualize: bool = False
    next_with_key: bool = False
    model_distance: float = 0.0


class DataPreprocessor(BaseNode):
    def __init__(self):
        self._params: DataPreprocessorParams = DataPreprocessorParams()  # pyright: ignore [reportIncompatibleVariableOverride]
        super().__init__(NODE_NAME)

        # Convert list -> np.ndarray
        if not isinstance(self._params.max_occlusion, np.ndarray):
            self._params.max_occlusion = np.array(self._params.max_occlusion).tolist()

        if len(self._params.max_occlusion) != 3:
            self.get_logger().error("Provided ROS parameter max_occlusion is invalid. Needs to have length of 3")
            self.destroy_node()
            return

        # Create main dir if required
        self.db_root_dir = Path(self._params.database_dir)
        self.get_logger().info(f"self.db_root_dir: {self.db_root_dir}")
        if not Path(self.db_root_dir).exists():
            self.get_logger().error("Database dir does not exist, did you generate samples first?")
            self.destroy_node()

        # Create sub_dir
        self.db_target_dir = Path(self.db_root_dir, "merged_samples")

        if self._params.visualize:
            self.gt_orig_pub = self.create_publisher(MarkerArray, ArculeeTopics.SAMPLE_COLLECTOR_ORIGINAL_GT, 1)
            self.gt_clean_pub = self.create_publisher(MarkerArray, ArculeeTopics.SAMPLE_COLLECTOR_CLEANED_GT, 1)
            self.pc_orig_pub = self.create_publisher(PointCloud2, ArculeeTopics.SAMPLE_COLLECTOR_ORIGINAL_POINTS, 1)
            self.pc_clean_pub = self.create_publisher(PointCloud2, ArculeeTopics.SAMPLE_COLLECTOR_CLEANED_POINTS, 1)
        else:
            self.gt_orig_pub = None
            self.gt_clean_pub = None
            self.pc_orig_pub = None
            self.pc_clean_pub = None

        self.get_logger().info("Initialized")

    def _read_pointcloud(self, file_path: Path) -> Any:
        return np.load(file_path).astype(np.float32).reshape(-1, 3)

    def _read_groundtruth(self, file_path: Path) -> List[BoundingBox3D]:
        lines = [line.rstrip() for line in open(file_path)]
        gt = [BoundingBox3D.from_txt_file(line) for line in lines]
        return gt

    def _copy_fraction_to_target(
        self, database: Any, db_dir: Path, db_path: Path, num_samples: int, cur_sample_id: Any, db_source_dir: Any
    ) -> Tuple[Any, int]:
        db_source_dir = Path(db_source_dir).parent

        self.get_logger().info(f"Received database: {database}")

        self.get_logger().info(f"PCFile column 1: {database.loc[1]['PCFile']}")

        num_skipped = 0
        # Combine for training set
        num_coll_total = 0
        for i in range(num_samples):
            # Select samples randomly
            sample_id = np.random.randint(len(database.index))

            # Get source paths
            gt_source_path = Path(db_source_dir, database.loc[sample_id]["GTFile"])
            pc_file_dict = ast.literal_eval(database.loc[sample_id]["PCFile"])
            pc_source_paths = {
                "combined": Path(db_source_dir, pc_file_dict["combined"]),
                "front": Path(db_source_dir, pc_file_dict["front"]),
                "rear": Path(db_source_dir, pc_file_dict["rear"]),
            }

            objects = self._read_groundtruth(gt_source_path)
            points = {key: self._read_pointcloud(pc_source_paths[key]) for key in pc_source_paths}

            # Check for min requirements
            if (
                points["combined"].shape[0] < self._params.min_points_sample
                or len(objects) < self._params.min_number_objects
            ):
                num_skipped += 0
                continue

            boxes_list = [object.box3d for object in objects]
            bboxes = np.array(boxes_list).reshape(-1, 7)

            if self._params.visualize:
                publish_gt_pc(
                    self.get_clock().now().to_msg(),
                    points=points["combined"],
                    groundtruth=bboxes,
                    pc_pub=self.pc_orig_pub,
                    gt_pub=self.gt_orig_pub,
                    colour=[1.0, 0.0, 0.0],
                    frame_id=self._params.frame_id,
                )
                # time.sleep(0.2)

            # Check for collisions
            boxes_coll = np.array(boxes_list).reshape(-1, 7)
            boxes_coll[:, 3:5] += self._params.model_distance
            box_corners = center_to_corner_box2d(boxes_coll[:, 0:2], boxes_coll[:, 3:5], boxes_coll[:, 6])
            collisions = box_collision_test(box_corners, box_corners)

            num_coll = np.sum(collisions)
            if num_coll > 0:
                self.get_logger().warn(f"Detected {num_coll} collisions!")
                num_coll_total += 1
                # continue

            class_names = np.array([object.type for object in objects]).reshape(-1)

            names = np.array(database.loc[sample_id]["ModelNames"].split(" "))

            # Remove occluded objects from pc and gt
            remove_points_mask_combined, keep_boxes_mask, occlusions = remove_occluded_objects(
                points["combined"], bboxes, self._params.max_occlusion, self._params.min_points_object
            )

            remove_points_mask_front, _, _ = remove_occluded_objects(
                points["front"], bboxes, self._params.max_occlusion, self._params.min_points_object
            )

            remove_points_mask_rear, _, _ = remove_occluded_objects(
                points["rear"], bboxes, self._params.max_occlusion, self._params.min_points_object
            )

            self.get_logger().info(f"occlusions: {occlusions}")
            self.get_logger().info(f"keep_boxes_mask: {keep_boxes_mask}")
            self.get_logger().info(f"remove_points_mask_combined: {remove_points_mask_combined}")
            self.get_logger().info(f"remove_points_mask_front: {remove_points_mask_front}")
            self.get_logger().info(f"remove_points_mask_rear: {remove_points_mask_rear}")

            cleaned_points = {
                "combined": points["combined"][~remove_points_mask_combined, :],
                "front": points["front"][~remove_points_mask_front, :],
                "rear": points["rear"][~remove_points_mask_rear, :],
            }
            cleaned_gt = bboxes[keep_boxes_mask, :]
            cleaned_types = class_names[keep_boxes_mask]
            cleaned_names = names[keep_boxes_mask]

            if np.any(remove_points_mask_combined) or not np.any(keep_boxes_mask):
                self.get_logger().warn("Found occluded objects!")
                points_diff = points["combined"].shape[0] - cleaned_points["combined"].shape[0]
                objects_diff = len(names) - len(cleaned_names)
                self.get_logger().warn(f"Removed {objects_diff} objects and {points_diff} points")

            if cleaned_points["combined"].shape[0] < self._params.min_points_sample:
                self.get_logger().warn(
                    f"Only {cleaned_points['combined'].shape[0]} points in sample, minimum required is: {self._params.min_points_sample}"
                )
                continue

            if self._params.visualize:
                publish_gt_pc(
                    self.get_clock().now().to_msg(),
                    points=cleaned_points["combined"],
                    groundtruth=cleaned_gt,
                    pc_pub=self.pc_clean_pub,
                    gt_pub=self.gt_clean_pub,
                    colour=[0.0, 1.0, 0.0],
                    frame_id=self._params.frame_id,
                    with_line=True,
                )

            store_sample_with_entry(
                cleaned_points,
                cleaned_gt,
                cleaned_types,
                cleaned_names,
                occlusions,
                cur_sample_id,
                db_dir,
                db_path,
                clean_names=False,
            )

            # Drop row and reset index
            database.drop([sample_id], inplace=True)
            database.reset_index(drop=True, inplace=True)

            cur_sample_id += 1

            progress_bar_update(i, first_total=num_samples, second=self.num_batch, second_total=self.num_batches)

        print(f"\nDetected {num_coll_total} samples with collisions.")
        return cur_sample_id, num_skipped

    def preprocess_generated_samples(self) -> None:
        self.get_logger().info("Start preprocessing")
        self.db_path = Path(self.db_target_dir, "sample_database.csv")

        training_dir = Path(self.db_target_dir, "training")
        testing_dir = Path(self.db_target_dir, "testing")

        # Create directories if required
        if not Path(training_dir).exists():
            self.db_target_dir.mkdir(parents=True, exist_ok=True)
            Path(training_dir, "points", "combined").mkdir(parents=True, exist_ok=True)
            Path(training_dir, "points", "front").mkdir(parents=True, exist_ok=True)
            Path(training_dir, "points", "rear").mkdir(parents=True, exist_ok=True)
            Path(training_dir, "groundtruth").mkdir(parents=True, exist_ok=True)
            Path(testing_dir, "points", "combined").mkdir(parents=True, exist_ok=True)
            Path(testing_dir, "points", "front").mkdir(parents=True, exist_ok=True)
            Path(testing_dir, "points", "rear").mkdir(parents=True, exist_ok=True)
            Path(testing_dir, "groundtruth").mkdir(parents=True, exist_ok=True)

        train_db_path = Path(training_dir, "sample_database.csv")
        test_db_path = Path(testing_dir, "sample_database.csv")

        databases = list(Path(self.db_root_dir).rglob("sample_database.csv"))
        db_list = []
        for db_path in sorted(databases):
            db_name = db_path.parent.name
            db_list.append((pd.read_csv(db_path), db_name, db_path))

        self.num_batches = 2 * len(db_list)
        self.num_batch = 0

        cur_id_train = 0
        cur_id_test = 0

        for db, db_name, db_path in db_list:
            num_samples_train = int(len(db.index) * self._params.training_split)
            num_samples_test = len(db.index) - num_samples_train

            self.num_batch += 1
            # Copy to training
            cur_id_train, num_skipped = self._copy_fraction_to_target(
                db, training_dir, train_db_path, num_samples_train, cur_id_train, db_path
            )
            self.get_logger().info(
                f"\nSkipped {num_skipped} on batch no. {self.num_batch} for training set {db_name}\n"
            )
            self.num_batch += 1

            # Copy remaining to testing
            cur_id_test, num_skipped = self._copy_fraction_to_target(
                db, testing_dir, test_db_path, num_samples_test, cur_id_test, db_path
            )
            self.get_logger().info(f"\nSkipped {num_skipped} on batch no. {self.num_batch} for testing set {db_name}\n")

        self.get_logger().info(
            f"Preprocessing finished, Training Samples: {cur_id_train + 1}, Testing Samples: {cur_id_test + 1}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = DataPreprocessor()
    node.preprocess_generated_samples()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
