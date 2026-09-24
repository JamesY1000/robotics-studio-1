import numpy as np
from typing import Any, Dict, Optional
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose

import ros2_numpy as rnp


class BoundingBox3D:
    def __init__(
        self,
        position: Any,
        box_size: Any,
        yaw: float,
        classification: Optional[int] = None,
        class_type: Optional[str] = None,
    ) -> None:
        if class_type is None:
            self.type = "UNCLASSIFIED"
        else:
            self.type = class_type

        if classification is None:
            self.classification = 0
        else:
            self.classification = classification
        self.t = position
        self.ry = yaw
        self.l = box_size[0]
        self.w = box_size[1]
        self.h = box_size[2]
        self.box3d = np.array([self.t[0], self.t[1], self.t[2], self.l, self.w, self.h, self.ry]).astype(np.float32)

    @classmethod
    def from_ros_msgs(cls, object_label: Any, pose: Pose, class_type_dict: Dict) -> "BoundingBox3D":
        classification = object_label.classification
        class_type = class_type_dict[classification]
        position = rnp.numpify(pose.position)
        quat = pose.orientation
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = euler[2]
        box_size = rnp.numpify(object_label.size)
        return cls(position, box_size, yaw, classification=classification, class_type=class_type)

    @classmethod
    def from_txt_file(cls, label_file_line: str) -> "BoundingBox3D":
        # classification truncated occluded alpha bboxLeft bboxTop bboxRight bboxBottom h w l x y z yaw score
        data = label_file_line.split(" ")
        class_type = data[0]
        height = float(data[8])
        width = float(data[9])
        length = float(data[10])
        x = float(data[11])
        y = float(data[12])
        z = float(data[13])
        yaw = float(data[14])
        position = np.array([x, y, z], dtype=np.float32)
        box_size = np.array([length, width, height], dtype=np.float32)
        return cls(position, box_size, yaw, class_type=class_type)

    def print_object(self) -> None:
        print(f"Type: {self.type}")
        print(f"3d bbox l,w,h: {self.l}, {self.w}, {self.h}")
        print(f"3d bbox location, ry: {self.t[0]}, {self.t[1]}, {self.t[2]}), {self.ry}")

    def to_numpy_array_with_type(self) -> np.ndarray:
        return np.array([self.type, self.t[0], self.t[1], self.t[2], self.l, self.w, self.h, self.ry])

    def to_numpy_array_2D(self) -> np.ndarray:
        return np.array([self.t[0], self.t[1], self.l, self.w, self.ry])
