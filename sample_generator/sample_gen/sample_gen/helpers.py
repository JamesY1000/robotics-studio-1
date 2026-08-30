import sys
import subprocess
import time
import traceback
from pathlib import Path
import csv
from string import digits
from typing import Any, Dict, List, Optional

import yaml
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point, Pose2D, Quaternion
from rclpy.client import Client
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory

from sample_gen.create_markers import create_bbox_marker, delete_marker_array
from rclpy.publisher import Publisher


DB_COLUMNS = ["ID", "PCFile", "GTFile", "NumObjects", "ModelNames"]


def store_database(
    database: List[Dict[str, Any]], path: Path, column_names: List[str], write_header: bool = True
) -> None:
    try:
        target_dir = Path(path).parent
        if not (target_dir.exists):
            target_dir.mkdir(parents=True, exist_ok=True)
        with path.open("w") as f:
            writer = csv.DictWriter(f, fieldnames=column_names)
            if write_header:
                writer.writeheader()
            for data in database:
                writer.writerow(data)
    except IOError:
        print("I/O error")
        print(traceback.format_exc())


def append_database(row: Dict[str, Any], path: Path, column_names: List[str], write_header: bool = False) -> None:
    try:
        with path.open("a") as f:
            writer = csv.DictWriter(f, fieldnames=column_names)
            if write_header:
                writer.writeheader()
            writer.writerow(row)
    except IOError:
        print("I/O error")
        print(traceback.format_exc())


def load_database(path: Path) -> List:
    try:
        database = []
        with path.open(mode="r") as f:
            column_names = next(csv.reader(f))
            reader = csv.DictReader(f, fieldnames=column_names)
            for row in reader:
                database.append(row)
            return database
    except Exception as e:
        print(f"Database not found ({e}), return empty list")
        return []


def get_pose_msg(
    x: float = 0.0, y: float = 0.0, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
) -> Pose:
    pose = Pose()
    pose.position = Point(x=float(x), y=float(y), z=float(z))
    quat = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return pose


def get_pose_from_pose_2d_msg(pose: Pose2D) -> Pose:
    pose_3d = Pose()
    pose_3d.position = Point(x=pose.x, y=pose.y, z=0.0)
    quat = quaternion_from_euler(0.0, 0.0, pose.theta)
    pose_3d.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return pose_3d


def get_pose_2d_msg(x: float = 0, y: float = 0, yaw: float = 0) -> Pose2D:
    pose = Pose2D()
    pose.x = x
    pose.y = y
    pose.theta = yaw
    return pose


def get_twist_msg(
    lin_x: float = 0.0,
    lin_y: float = 0.0,
    lin_z: float = 0.0,
    ang_x: float = 0.0,
    ang_y: float = 0.0,
    ang_z: float = 0.0,
) -> Twist:
    twist = Twist()
    twist.linear.x = float(lin_x)
    twist.linear.y = float(lin_y)
    twist.linear.z = float(lin_z)
    twist.angular.x = float(ang_x)
    twist.angular.y = float(ang_y)
    twist.angular.z = float(ang_z)
    return twist


def progress_bar_update(
    first: int, first_total: Optional[int] = None, second: Optional[int] = None, second_total: Optional[int] = None
) -> None:
    if first_total is not None:
        current = first / first_total * 100
    else:
        current = 100
    indicator = int(current / 10)
    sys.stdout.write("\r")
    if second is not None and first_total is not None:
        sys.stdout.write(
            f"[{'=' * indicator:<10}] {current}% ({first} of {first_total}) (Batch {second} of {second_total})"
        )
    elif first_total is None and second is not None and second_total is not None:
        sys.stdout.write(
            f"[{'=' * indicator:<10}] {current}% (Collected {first} samples) (Pos {second} of {second_total})"
        )
    elif first_total is None:
        sys.stdout.write(f"[{'=' * indicator:<10}] {current}% (Collected {first} samples)")
    else:
        sys.stdout.write(f"[{'=' * indicator:<10}] {current}% ({first} of {first_total})")
    sys.stdout.flush()


def array_to_pc2(points: np.ndarray, frame_id: str, stamp: Any) -> PointCloud2:
    """Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions (m)
        frame_id: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    cloud_msg = sensor_msgs.PointCloud2()
    cloud_msg.header = std_msgs.Header(frame_id=frame_id, stamp=stamp)
    cloud_msg.height = 1
    cloud_msg.width = points.shape[0]
    cloud_msg.fields = [
        sensor_msgs.PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1) for i, n in enumerate("xyz")
    ]
    cloud_msg.is_bigendian = False  # assumption
    cloud_msg.point_step = points.dtype.itemsize * 3
    cloud_msg.row_step = cloud_msg.point_step * points.shape[0]
    # cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.is_dense = False
    cloud_msg.data = points.astype(dtype).tobytes()

    return cloud_msg


def publish_gt_pc(
    stamp: Any,
    points: np.ndarray = np.ndarray([]),
    groundtruth: np.ndarray = np.ndarray([]),
    gt_pub: Optional[Publisher] = None,
    pc_pub: Optional[Publisher] = None,
    frame_id: str = "world",
    colour: List[float] = [1.0, 0.0, 0.0],
    ns: str = "default",
    with_line: bool = False,
    with_box: bool = True,
) -> None:
    if pc_pub is not None and len(points) > 0:
        pc2_msg = array_to_pc2(points, frame_id, stamp)
        pc_pub.publish(pc2_msg)

    if gt_pub is not None and len(groundtruth) > 0:
        box_markers = MarkerArray()
        box_markers.markers = []
        delete_marker_array(frame_id, gt_pub)
        for i in range(groundtruth.shape[0]):
            create_bbox_marker(
                bbox=groundtruth[i, :],
                frame_id=frame_id,
                marker_array=box_markers,
                colour=colour,
                ns=ns,
                with_line=with_line,
                with_box=with_box,
            )

        gt_pub.publish(box_markers)


def create_ros_service_client(topic: str, service_type: Any, node: Node) -> Client:
    node.get_logger().info(f"Creating Service client to connect to {topic}")
    client = node.create_client(service_type, topic)
    if not client.service_is_ready():
        node.get_logger().info("Waiting for service to become available...")
        client.wait_for_service()
        node.get_logger().info(f"...connected to {topic} service!")
    return client


def load_yaml(path: Path) -> Any:
    path = Path(path)
    if path.is_file():
        try:
            with path.open("r") as f:
                return yaml.load(f, Loader=yaml.FullLoader)
        except Exception:
            return None


def load_yaml_entry(key: str, pkg_name: str, config_file: str = "config.yml") -> dict:
    pkg_dir = Path(get_package_share_directory(pkg_name))
    config_path = pkg_dir / "config" / config_file
    config_yml = load_yaml(config_path)
    if config_yml and key in config_yml:
        return config_yml[key]
    else:
        raise KeyError(f"Key '{key}' not found in YAML file")


def store_sample_with_entry(
    points: Dict[str, np.ndarray],
    groundtruth: np.ndarray,
    classifications: np.ndarray,
    model_names: np.ndarray,
    occlusions: List,
    cur_sample_id: int,
    db_dir: Path,
    db_path: Path,
    clean_names: bool = True,
) -> str:
    sample_name = f"{cur_sample_id}"

    pc_paths = {
        "combined": Path("points", "combined", sample_name),
        "front": Path("points", "front", sample_name),
        "rear": Path("points", "rear", sample_name),
    }

    for key in pc_paths:
        pc_path = Path(db_dir, pc_paths[key])
        pc_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(pc_path, points[key])

    gt_path_short = Path("groundtruth", f"{sample_name}.txt")
    gt_path = Path(db_dir, gt_path_short)

    with gt_path.open("w") as f:
        for i in range(groundtruth.shape[0]):  # x, y, z, l, w, h, yaw
            classification = classifications[i]
            truncated = 0.0  # float: from 0 (non-truncated) to 1 (truncated), where truncated refers to the object leaving image boundaries | Not used
            occluded = occlusions[
                i
            ]  # int: indicating occlusion state: 0 = fully visible, 1 = partly occluded, 2 = largely occluded, 3 = unknown | Not used
            alpha = 0.0  # float: Observation angle of object, ranging [-pi..pi] | Not used
            bbox_left = 0.0  # float: Pixel coords of 2D bbox in camera image | Not used
            bbox_top = 0.0
            bbox_right = 0.0
            bbox_bottom = 0.0
            x = groundtruth[i, 0]
            y = groundtruth[i, 1]
            z = groundtruth[i, 2]
            length = groundtruth[i, 3]
            w = groundtruth[i, 4]
            h = groundtruth[i, 5]
            yaw = groundtruth[i, 6]
            # classification truncated occluded alpha bbox_left bbox_top bbox_right bbox_bottom h w l x y z yaw score
            f.write(
                f"{classification} {truncated:.2f} {occluded} {alpha:.2f} {bbox_left:.2f} {bbox_top:.2f} {bbox_right:.2f} {bbox_bottom:.2f} {h:.2f} {w:.2f} {length:.2f} {x:.2f} {y:.2f} {z:.2f} {yaw:.2f}\n"
            )

    # Store entry and database
    sample_entry = dict()
    sample_entry["ID"] = cur_sample_id
    sample_entry["PCFile"] = {key: str(pc_paths[key]) + ".npy" for key in pc_paths}
    sample_entry["GTFile"] = gt_path_short
    sample_entry["NumObjects"] = groundtruth.shape[0]
    # Remove non relevant characters required for simulation
    if clean_names:
        cleaned_names = []
        for name in model_names:
            cleanedName = name.rstrip(digits)[:-1]
            cleanedName = cleanedName.rstrip(digits)[:-1]
            cleaned_names.append(cleanedName)
        sample_entry["ModelNames"] = " ".join(name for name in cleaned_names)  # Remove non relevant characters
    else:
        sample_entry["ModelNames"] = " ".join(name for name in model_names)  # Remove non relevant characters

    if cur_sample_id == 0:
        write_header = True
    else:
        write_header = False

    append_database(sample_entry, db_path, DB_COLUMNS, write_header)
    return sample_name


# TODO: (james) Return the success of these gz service calls


def spawn_entity_process(
    node: Node, world_name: str, model_name: str, sdf_filename: str, spawn_pose: Optional[Pose] = None
):
    if spawn_pose:
        pose = (
            f"{{ position: {{ x: {spawn_pose.position.x}, y: {spawn_pose.position.y}, z: {spawn_pose.position.z} }}, "
            f"orientation: {{ w: {spawn_pose.orientation.w}, x: {spawn_pose.orientation.x}, y: {spawn_pose.orientation.y}, z: {spawn_pose.orientation.z} }} }}"
        )
        request = f'sdf_filename: "{sdf_filename}",' f'name: "{model_name}",' f"pose: {pose}"

    else:
        request = f'sdf_filename: "{sdf_filename}",' f'name: "{model_name}"'

    command = [
        "ign",
        "service",
        "-s",
        f"/world/{world_name}/create",
        "--reqtype",
        "ignition.msgs.EntityFactory",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "10000",
        "--req",
        request,
    ]

    # `ign service` can return exit code 0 with an empty/failed response when
    # the server is still starting up (the 1 s default timeout was too short).
    # Only treat it as success when the reply actually contains "true".
    for attempt in range(1, 4):
        result = subprocess.run(command, capture_output=True, text=True)
        response = (result.stdout + result.stderr).strip().lower()
        node.get_logger().info(f"spawn {model_name} attempt {attempt}: rc={result.returncode} resp={response!r}")
        if result.returncode == 0 and "true" in response:
            return True
    node.get_logger().error(f"Failed to spawn {model_name} after retries: {response}")
    return False


def delete_entity_process(node: Node, world_name: str, model_name: str):
    request = f'name: "{model_name}",' f"type: MODEL"

    command = [
        "ign",
        "service",
        "-s",
        f"/world/{world_name}/remove",
        "--reqtype",
        "ignition.msgs.Entity",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "1000",
        "--req",
        request,
    ]

    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode == 0:
        node.get_logger().info(f"Service call to /world/{world_name}/remove successful: {result.stdout}")
        return True
    else:
        node.get_logger().info(f"Service call to /world/{world_name}/remove failed: {result.stderr}")
        return False


def set_pose_process(node: Node, set_pose: Pose, world_name: str, model_name: str):
    request = (
        f'name: "{model_name}", '
        f"position: {{x: {set_pose.position.x}, y: {set_pose.position.y}, z: {set_pose.position.z}}}, "
        f"orientation: {{x: {set_pose.orientation.x}, y: {set_pose.orientation.y}, z: {set_pose.orientation.z}, w: {set_pose.orientation.w}}}"
    )

    command = [
        "ign",
        "service",
        "-s",
        f"/world/{world_name}/set_pose",
        "--reqtype",
        "ignition.msgs.Pose",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        request,
    ]

    # The set_pose service returns "true" even when it cannot find the entity by
    # name (e.g. if the entity registration from a recent create is still in
    # flight), so retry a few times to ride out that race.
    for attempt in range(1, 4):
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode == 0 and "true" in (result.stdout + result.stderr).lower():
            node.get_logger().info(f"Service call to /world/{world_name}/set_pose successful: {result.stdout.strip()}")
            return True
        node.get_logger().info(f"set_pose {model_name} attempt {attempt}: rc={result.returncode} out={result.stdout.strip()!r}")
        time.sleep(0.5)
    node.get_logger().error(f"Service call to /world/{world_name}/set_pose failed for {model_name}")
    return False
