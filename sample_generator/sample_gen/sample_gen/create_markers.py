from typing import Any, Dict, List, Optional, Union

import numpy as np
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.publisher import Publisher
from scipy.spatial.transform import Rotation


def create_spawn_area_marker(
    frame_id: str, spawn_area: list[tuple[float, float]], size: float = 0.1, colour: List[float] = [0.0, 0.0, 1.0]
) -> Marker:
    spawnAreaMarker = Marker()
    spawnAreaMarker.ns = "model_area_marker"
    spawnAreaMarker.header.frame_id = frame_id
    spawnAreaMarker.type = Marker.LINE_STRIP
    spawnAreaMarker.action = Marker.ADD
    spawnAreaMarker.pose.position.x = 0.0
    spawnAreaMarker.pose.position.y = 0.0
    spawnAreaMarker.pose.position.z = 0.0
    spawnAreaMarker.pose.orientation.x = 0.0
    spawnAreaMarker.pose.orientation.y = 0.0
    spawnAreaMarker.pose.orientation.z = 0.0
    spawnAreaMarker.pose.orientation.w = 1.0
    spawnAreaMarker.scale.x = size
    spawnAreaMarker.points = []
    spawnAreaMarker.color.a = 1.0
    spawnAreaMarker.color.r = colour[0]
    spawnAreaMarker.color.g = colour[1]
    spawnAreaMarker.color.b = colour[2]

    previousPoint = None
    firstPoint = None
    for point in spawn_area:
        next_point = Point()
        next_point.x = float(point[0])
        next_point.y = float(point[1])
        next_point.z = 0.0
        if previousPoint is not None:
            # Add line segment between last point and new point
            spawnAreaMarker.points.append(previousPoint)
            spawnAreaMarker.points.append(next_point)

        if firstPoint is None:
            firstPoint = next_point

        previousPoint = next_point

    # Close polygon
    spawnAreaMarker.points.append(next_point)
    spawnAreaMarker.points.append(firstPoint)

    return spawnAreaMarker


def create_status_marker(frame_id: str, sample: int, epoch: int, loss: float) -> Marker:
    # Marker for label and score
    marker_label = Marker()
    marker_label.id = 1
    marker_label.header.frame_id = frame_id
    marker_label.type = Marker.TEXT_VIEW_FACING
    marker_label.action = Marker.ADD
    # Place centered on top of bBox top plane
    marker_label.pose.position.x = -10
    marker_label.pose.position.y = 0
    marker_label.pose.position.z = 0
    marker_label.scale.z = 2
    marker_label.color.a = 1.0
    marker_label.color.r = 0.0
    marker_label.color.g = 0.0
    marker_label.color.b = 0.8
    # Add ID
    marker_label.text = "Epoch: {} Batches: {} Loss: {:.2f}".format(epoch, sample, loss)

    return marker_label


def delete_marker_array(frame_id: str, publisher: Publisher, ns: str = "default") -> None:
    markers = MarkerArray()
    marker = Marker()
    markers.markers = []  # Initialised to satisfy pyright AttributeAccessIssue
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.action = Marker.DELETEALL
    markers.markers.append(marker)
    publisher.publish(markers)


def delete_selected_markers(publisher: Publisher, ids: List[int], frame_id: str = "world", ns: str = "default") -> None:
    markers = MarkerArray()
    markers.markers = []
    for id in ids:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.id = id
        marker.action = Marker.DELETE
        markers.markers.append(marker)

    publisher.publish(markers)


def create_bbox_marker(
    bbox: Any,
    frame_id: str,
    marker_array: MarkerArray,
    label: Optional[str] = None,
    class_names: Optional[Dict] = None,
    score: Optional[float] = None,
    with_box: bool = True,
    with_line: bool = False,
    colour: List[float] = [0.58, 0.58, 0.58],
    a: float = 0.2,
    lifetime: Optional[int] = None,
    two_dim: bool = False,
    ns: str = "default",
) -> None:
    index = len(marker_array.markers)
    marker_array.markers = []

    # Extract info from bbox array
    if two_dim:
        box_pos_x = float(bbox[0])
        box_pos_y = float(bbox[1])
        box_pos_z = 0.0
        box_size_x = float(bbox[2])
        box_size_y = float(bbox[3])
        box_size_z = 1.0
        box_yaw = float(bbox[4])
    else:
        box_pos_x = float(bbox[0])
        box_pos_y = float(bbox[1])
        box_pos_z = float(bbox[2])
        box_size_x = float(bbox[3])
        box_size_y = float(bbox[4])
        box_size_z = float(bbox[5])
        box_yaw = float(bbox[6])

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [0, 0, box_yaw], degrees=False)

    # Convert to quaternions and print
    quat = rot.as_quat(canonical=False)

    # Marker for Box
    if with_box:
        marker_box = Marker()
        marker_box.id = index
        marker_box.ns = ns
        marker_box.header.frame_id = frame_id
        marker_box.type = Marker.CUBE
        marker_box.action = Marker.ADD
        if lifetime is not None:
            marker_box.lifetime = Duration(seconds=lifetime)
        marker_box.pose.position.x = box_pos_x
        marker_box.pose.position.y = box_pos_y
        marker_box.pose.position.z = box_pos_z
        marker_box.pose.orientation.x = quat[0]
        marker_box.pose.orientation.y = quat[1]
        marker_box.pose.orientation.z = quat[2]
        marker_box.pose.orientation.w = quat[3]
        marker_box.scale.x = box_size_x
        marker_box.scale.y = box_size_y
        marker_box.scale.z = box_size_z
        marker_box.color.a = a
        marker_box.color.r = colour[0]
        marker_box.color.g = colour[1]
        marker_box.color.b = colour[2]
        marker_array.markers.append(marker_box)

    if label is not None or score is not None:
        # Marker for label and score
        marker_label = Marker()
        marker_label.id = index + 1
        marker_label.ns = ns
        marker_label.header.frame_id = frame_id
        marker_label.type = Marker.TEXT_VIEW_FACING
        marker_label.action = Marker.ADD
        if lifetime is not None or lifetime is False:
            marker_label.lifetime = Duration(seconds=lifetime)
        # Place centered on top of bBox top plane
        marker_label.pose.position.x = box_pos_x
        marker_label.pose.position.y = box_pos_y
        marker_label.pose.position.z = box_pos_z + 0.5 * abs(box_size_z)
        marker_label.scale.z = 2
        marker_label.color.a = 1.0
        marker_label.color.r = 0.58
        marker_label.color.g = 0.58
        marker_label.color.b = 0.58
        # Add Label
        if class_names is not None:
            if isinstance(label, str):
                label = label
            else:
                label = class_names[label]
        if score is None:
            marker_label.text = "ID: {}".format(label)
        else:
            marker_label.text = "Class: {}\n Score: {:.2f}".format(label, score)
        marker_array.markers.append(marker_label)

    if with_line:
        # Marker for outlines
        marker_line = Marker()
        if label is not None or score is not None:
            marker_line.id = index + 2
        else:
            marker_line.id = index + 1
        marker_line.header.frame_id = frame_id
        marker_line.ns = ns
        marker_line.type = Marker.LINE_LIST
        marker_line.action = Marker.ADD
        if lifetime is not None or lifetime is False:
            marker_line.lifetime = Duration(seconds=lifetime)
        marker_line.pose.position.x = box_pos_x
        marker_line.pose.position.y = box_pos_y
        marker_line.pose.position.z = box_pos_z
        marker_line.pose.orientation.x = quat[0]
        marker_line.pose.orientation.y = quat[1]
        marker_line.pose.orientation.z = quat[2]
        marker_line.pose.orientation.w = quat[3]
        marker_line.scale.x = 0.05
        marker_line.color.a = 1.0
        marker_line.color.r = 1.0
        marker_line.color.g = 1.0
        marker_line.color.b = 1.0
        # Calculate Corner Points
        b_low_left = Point()
        b_low_left.x = -0.5 * box_size_x
        b_low_left.y = -0.5 * box_size_y
        b_low_left.z = -0.5 * box_size_z
        b_low_right = Point()
        b_low_right.x = 0.5 * box_size_x
        b_low_right.y = -0.5 * box_size_y
        b_low_right.z = -0.5 * box_size_z
        b_up_right = Point()
        b_up_right.x = 0.5 * box_size_x
        b_up_right.y = 0.5 * box_size_y
        b_up_right.z = -0.5 * box_size_z
        b_up_left = Point()
        b_up_left.x = -0.5 * box_size_x
        b_up_left.y = 0.5 * box_size_y
        b_up_left.z = -0.5 * box_size_z
        t_low_left = Point()
        t_low_left.x = b_low_left.x
        t_low_left.y = b_low_left.y
        t_low_left.z = 0.5 * box_size_z
        t_low_right = Point()
        t_low_right.x = b_low_right.x
        t_low_right.y = b_low_right.y
        t_low_right.z = 0.5 * box_size_z
        t_up_right = Point()
        t_up_right.x = b_up_right.x
        t_up_right.y = b_up_right.y
        t_up_right.z = 0.5 * box_size_z
        t_up_left = Point()
        t_up_left.x = b_up_left.x
        t_up_left.y = b_up_left.y
        t_up_left.z = 0.5 * box_size_z
        # Pillars
        # lowleft
        marker_line.points = []
        marker_line.points.append(b_low_left)
        marker_line.points.append(t_low_left)
        marker_line.points.append(b_low_right)
        marker_line.points.append(t_low_right)
        marker_line.points.append(b_up_left)
        marker_line.points.append(t_up_left)
        marker_line.points.append(b_up_right)
        marker_line.points.append(t_up_right)
        # Ground plane
        marker_line.points.append(b_low_left)
        marker_line.points.append(b_low_right)
        marker_line.points.append(b_low_left)
        marker_line.points.append(b_up_left)
        marker_line.points.append(b_up_left)
        marker_line.points.append(b_up_right)
        marker_line.points.append(b_low_right)
        marker_line.points.append(b_up_right)
        # Top plane
        marker_line.points.append(t_low_left)
        marker_line.points.append(t_low_right)
        marker_line.points.append(t_low_left)
        marker_line.points.append(t_up_left)
        marker_line.points.append(t_up_left)
        marker_line.points.append(t_up_right)
        marker_line.points.append(t_low_right)
        marker_line.points.append(t_up_right)
        marker_array.markers.append(marker_line)


def create_box_corner_marker(
    box_corners: np.ndarray,
    frame_id: str,
    marker_array: MarkerArray,
    colour: List[float] = [1.0, 0.0, 0.0],
    lifetime: Optional[Union[int, float]] = None,
    ns: str = "default",
) -> None:
    index = len(marker_array.markers)

    # Marker for outlines
    marker_line = Marker()
    marker_line.id = index
    marker_line.ns = ns
    marker_line.header.frame_id = frame_id
    marker_line.type = Marker.LINE_LIST
    marker_line.action = Marker.ADD
    if lifetime:
        marker_line.lifetime = Duration(seconds=lifetime)
    marker_line.pose.position.x = 0.0
    marker_line.pose.position.y = 0.0
    marker_line.pose.position.z = 0.0
    marker_line.pose.orientation.x = 0.0
    marker_line.pose.orientation.y = 0.0
    marker_line.pose.orientation.z = 0.0
    marker_line.pose.orientation.w = 1.0
    marker_line.scale.x = 0.1
    marker_line.color.a = 1.0
    marker_line.color.r = colour[0]
    marker_line.color.g = colour[1]
    marker_line.color.b = colour[2]

    # Calculate Corner Points
    lowLeft = Point()
    lowLeft.x = box_corners[1][0]
    lowLeft.y = box_corners[1][1]
    lowLeft.z = 0.0
    lowRight = Point()
    lowRight.x = box_corners[0][0]
    lowRight.y = box_corners[0][1]
    lowRight.z = 0.0
    upRight = Point()
    upRight.x = box_corners[3][0]
    upRight.y = box_corners[3][1]
    upRight.z = 0.0
    upLeft = Point()
    upLeft.x = box_corners[2][0]
    upLeft.y = box_corners[2][1]
    upLeft.z = 0.0

    # Create lines from points
    marker_line.points = []
    marker_line.points.append(lowLeft)
    marker_line.points.append(lowRight)

    marker_line.points.append(lowRight)
    marker_line.points.append(upRight)

    marker_line.points.append(upRight)
    marker_line.points.append(upLeft)

    marker_line.points.append(upLeft)
    marker_line.points.append(lowLeft)

    marker_array.markers = []
    marker_array.markers.append(marker_line)
