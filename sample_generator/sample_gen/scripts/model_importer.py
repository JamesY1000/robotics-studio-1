#!/usr/bin/env python3

import traceback
import time
from typing import Any
from pathlib import Path
from dataclasses import dataclass, field

import rclpy
from rclpy.client import Client

import rclpy.parameter
from std_srvs.srv import Trigger


from sample_gen.base import BaseNode, BaseParams
from sample_gen.services import (
    SERVICE_CHECK_MODELS_LOADED,
    SERVICE_GET_GT_DATA,
    SERVICE_UPDATE_GT_SUBS,
)
from sample_gen_msgs.srv import CheckModelsLoaded, GetDBData
from sample_gen.helpers import (
    create_ros_service_client,
    get_pose_msg,
    load_yaml_entry,
    store_database,
    spawn_entity_process,
    delete_entity_process,
)
from ament_index_python.packages import get_package_share_directory

NODE_NAME = "model_importer_node"
MODEL_DB_COLUMNS = [
    "ID",
    "Name",
    "Classification",
    "Type",
    "BboxLength",
    "BboxWidth",
    "BboxHeight",
    "OffsetX",
    "OffsetY",
    "OffsetZ",
    "ModelPath",
]

SAMPLE_GEN_PKG = "sample_gen"
SAMPLE_GEN_DIR = Path(get_package_share_directory("sample_gen"))
CONFIG_PATH = SAMPLE_GEN_DIR / "config" / "config.yml"


@dataclass
class ModelImporterParams(BaseParams):
    database_dir: str = "/brain2/ros2/ros2_ws/install/sample_gen/share/sample_gen/database"
    world_name: str = ""
    frame_id: str = ""
    model_names: list[str] = field(default_factory=lambda: [""])
    class_config_yml: str = "class_configs.yml"
    class_config_name: str = "class_config"
    class_config_dict_name: str = "classes"  # TODO: (james) Is this necessary?
    model_dir: str = ""


@dataclass
class GTData:
    length: float
    width: float
    height: float
    offset_x: float
    offset_y: float
    offset_z: float
    classification: str | int
    class_name: str


SAMPLE_GEN_PK = "sample_gen"


class ModelImporter(BaseNode):
    def __init__(self):
        self._params: ModelImporterParams = ModelImporterParams()  # pyright: ignore [reportIncompatibleVariableOverride]
        self._gt_data = GTData
        super().__init__(NODE_NAME)

        self.world_name = self._params.world_name

        self.class_config = load_yaml_entry(
            self._params.class_config_name, SAMPLE_GEN_PKG, self._params.class_config_yml
        )

        self.get_logger().debug("Starting model import! getting class_configs now...")

        if self.class_config:
            self.class_config_dict = self.class_config[self._params.class_config_dict_name]
            self.class_id_dict = {x["class"]: x["id"] for x in self.class_config_dict if "class" and "id" in x}
        else:
            self.get_logger().error("Class config parameter is empty, node will not be able to check scaling")

        self.get_logger().debug(f"Class config got and class_id_dict extracted! Contents: {self.class_id_dict}")

        # Create folders and database

        self.model_db_path = Path(self._params.database_dir, "model_db.csv")
        self.model_db_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            self.check_models_loaded_client = create_ros_service_client(
                SERVICE_CHECK_MODELS_LOADED, CheckModelsLoaded, self
            )

            self.get_gt_data_client = create_ros_service_client(SERVICE_GET_GT_DATA, GetDBData, self)

            self.update_gt_subs_client = create_ros_service_client(SERVICE_UPDATE_GT_SUBS, Trigger, self)

        except Exception as e:
            self.get_logger().info(f"Error: {e}")

        self.spawn_entity_service_name: str = f"world/{self._params.world_name}/create"
        self.delete_entity_service_name: str = f"world/{self._params.world_name}/remove"

        self.get_logger().info("Initialized")

    def _call_service(self, client: Client, request: Any) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _get_gt_data(self) -> Any:
        self.get_logger().debug(f"Sending service request to {SERVICE_GET_GT_DATA}")
        request = GetDBData.Request()
        request.stamp = self.get_clock().now().to_msg()
        while True:
            response = self._call_service(self.get_gt_data_client, request)
            if response.success is True:
                return response
            self.get_logger().debug("Get GT Data not successful, wait 10ms")
            time.sleep(0.1)

    def _check_models_loaded_request(self, num_models: int = 1) -> Any:
        self.get_logger().debug(f"Sending service request to {SERVICE_CHECK_MODELS_LOADED}")
        request = CheckModelsLoaded.Request()
        request.num_models = num_models
        response = self._call_service(self.check_models_loaded_client, request)
        return response.success

    def run_model_import(self) -> None:
        # Make sure to shutdown everything if an error occurs

        try:
            current_id = 0
            model_database = []
            num_models = len(self._params.model_names)

            for model_name in self._params.model_names:
                self.get_logger().info(f"Getting info of model {current_id}/{num_models}: {model_name}")
                model_path: Path = Path(self._params.model_dir, model_name)
                saved_model_path: Path = Path(model_path, "model.sdf")

                # Spawn model and get bounding box information from objectState
                spawn_entity_process(
                    node=self,
                    world_name=self._params.world_name,
                    model_name=model_name,
                    sdf_filename=str(saved_model_path),
                    spawn_pose=get_pose_msg(),
                )
                self.get_logger().info(f"Spawned model '{model_name}' into world '{self._params.world_name}'")

                self.get_logger().info(f"Sending service request to {SERVICE_UPDATE_GT_SUBS}")
                if not self._call_service(self.update_gt_subs_client, Trigger.Request()):
                    self.get_logger().info("Failed to update gt subscriptions")
                else:
                    self.get_logger().info("Successfully updated gt subscriptions")

                while not self._check_models_loaded_request():
                    self.get_logger().info(
                        "Check models loaded service failed..."
                    )  # TODO: (james) REMOVE AFTER TESTING

                    self.get_logger().info(f"Sending service request to {SERVICE_UPDATE_GT_SUBS}")
                    self._call_service(self.update_gt_subs_client, Trigger.Request())
                    time.sleep(0.5)
                self.get_logger().info("Model successfully loaded")

                gt_data = self._get_gt_data()
                if gt_data is not None:
                    self._gt_data.length = gt_data.size.x
                    self._gt_data.width = gt_data.size.y
                    self._gt_data.height = gt_data.size.z
                    self._gt_data.offset_x = gt_data.offset.x
                    self._gt_data.offset_y = gt_data.offset.y
                    self._gt_data.offset_z = gt_data.offset.z
                    self._gt_data.classification = gt_data.classification
                    self._gt_data.class_name = gt_data.class_name
                else:
                    return

                time.sleep(1.0)

                # make a call to delete_entity subprocess
                delete_entity_process(node=self, world_name=self._params.world_name, model_name=model_name)

                # Wait (bounded) until the collector reports 0 models before
                # spawning the next one. The deleted model's ObjectLabel plugin
                # can keep publishing briefly (async teardown + DDS
                # unadvertisement), so its stale labels would otherwise be
                # mistaken for the next model's ground truth.
                self._call_service(self.update_gt_subs_client, Trigger.Request())
                for _ in range(50):  # up to ~10 s
                    if self._check_models_loaded_request(0):
                        break
                    self.get_logger().info("Waiting for previous model to be removed from ground truth...")
                    self._call_service(self.update_gt_subs_client, Trigger.Request())
                    time.sleep(0.2)

                model_entry = dict()
                model_entry["ID"] = current_id
                model_entry["Name"] = model_name
                model_entry["Classification"] = self._gt_data.classification
                model_entry["Type"] = self._gt_data.class_name
                model_entry["BboxLength"] = round(self._gt_data.length, 3)
                model_entry["BboxWidth"] = round(self._gt_data.width, 3)
                model_entry["BboxHeight"] = round(self._gt_data.height, 3)
                model_entry["OffsetX"] = round(self._gt_data.offset_x, 3)
                model_entry["OffsetY"] = round(self._gt_data.offset_y, 3)
                model_entry["OffsetZ"] = round(self._gt_data.offset_z, 3)
                model_entry["ModelPath"] = saved_model_path
                model_database.append(model_entry)
                current_id += 1

            store_database(model_database, self.model_db_path, MODEL_DB_COLUMNS)
            shutDownReason = f"Finished with all models, collected info of {current_id} models in total"
        except Exception:
            self.get_logger().warn(traceback.format_exc())
            shutDownReason = "Issue during generation of model database, shutting down everything."

        self.get_logger().info(shutDownReason)

        self.get_logger().info("testing service connections complete")


def main(args=None):
    rclpy.init(args=args)

    node = ModelImporter()
    node.run_model_import()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
