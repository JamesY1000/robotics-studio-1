#!/usr/bin/env python3

from pathlib import Path

from ament_index_python import get_package_share_directory
from dataclasses import dataclass

import rclpy
import rclpy.parameter
import rclpy.qos

import numpy as np
import cv2

try:
    from arculus_common import BaseNode, BaseParams  # type: ignore
except ImportError:
    from sample_gen.base import BaseNode, BaseParams


"""Projects 3D pointclouds into 2D img format"""

NODE_NAME = "yolo_format_node"
IMG_SIZE = 1280
POINT_COLOUR = 255
SAMPLE_GEN_PKG = "sample_gen"
MERGED_SAMPLES = Path(get_package_share_directory(SAMPLE_GEN_PKG), "database", "merged_samples")
BOUNDS = 5


# TODO: (james) Pass in the path files as rosparameters
@dataclass
class YoloFormatParams(BaseParams):
    merged_samples_path: str = "/brain2/ros2/ros2_ws/install/sample_gen/share/sample_gen/database/merged_samples"


class YoloFormat(BaseNode):
    def __init__(self):
        self._params = YoloFormatParams()
        super().__init__(NODE_NAME)

        self.get_logger().info("Initialised")

    # TODO: (james) Change groundtruth to oriented bounding box format
    # Required format: class_id center_x center_y width height keypoint1_x keypoint1_y keypoint2_x keypoint2_y ... keypointN_x keypointN_y

    def generate_yolo_data(self):
        yolo_dir = Path(MERGED_SAMPLES, "yolo")

        # Make directories in yolo format
        for view in ["combined", "front", "rear"]:
            for split in ["train", "val"]:
                Path(yolo_dir, view, "images", split).mkdir(parents=True, exist_ok=True)
                Path(yolo_dir, view, "labels", split).mkdir(parents=True, exist_ok=True)

        for folder in ["testing", "training"]:
            if Path(MERGED_SAMPLES, folder).is_dir():
                sensor_folder = Path(MERGED_SAMPLES, folder, "points")  # [testing/points, training/points]
                self.get_logger().info(f"Processing folder: {sensor_folder}")
                for pointcloud_folder in sensor_folder.iterdir():  # [combined, front, rear]
                    self.get_logger().info(f"Processing folder: {pointcloud_folder}")

                    for sample_points in pointcloud_folder.glob("*.npy"):  # [0.npy, 1.npy, 2.npy...]
                        points = np.load(sample_points)

                        # Convert pc to img
                        pc_img, scale = self.generate_point_cloud_image(points, IMG_SIZE, bounds=BOUNDS)

                        # Apply mask to remove points of the robot itself
                        pc_img = self.apply_mask(pc_img, 8)

                        # Read ground truth file
                        gt_file = Path(MERGED_SAMPLES, folder, "groundtruth", sample_points.stem + ".txt")
                        if gt_file.exists():
                            # self.get_logger().info("GT file exists")
                            with open(gt_file, "r") as f:
                                lines = f.readlines()
                            formatted_lines = []

                            for line in lines:
                                parts = line.strip().split()

                                # TODO: (james) Fix this in sample_gen parameters - replace 'BOSCH_TROLLEY' with corresponding int
                                # class_id = parts[0]
                                class_id = 0

                                bbox_height = float(parts[8])
                                bbox_width = float(parts[9])
                                bbox_centre_x = float(parts[11])
                                bbox_centre_y = float(parts[12])
                                bbox_centre_yaw = float(parts[14])

                                # Convert world coordinates (x, y) into pixel space
                                bbox_centre_img_x = int((bbox_centre_x + BOUNDS) * scale)
                                bbox_centre_img_y = int((bbox_centre_y + BOUNDS) * scale)

                                # Normalize bbox parameters for YOLO
                                norm_bbox_centre_px_x = bbox_centre_img_x / IMG_SIZE
                                norm_bbox_centre_px_y = 1 - (
                                    bbox_centre_img_y / IMG_SIZE
                                )  # Flip Y axis for image format

                                norm_bbox_width = (bbox_width * scale) / IMG_SIZE
                                norm_bbox_height = (bbox_height * scale) / IMG_SIZE

                                # Calculate the four corners of the bounding box
                                corners = self.calculate_bbox_corners(
                                    bbox_centre_x, bbox_centre_y, bbox_width, bbox_height, bbox_centre_yaw, scale
                                )

                                # Normalize the corners for YOLO
                                norm_corners = [(x / IMG_SIZE, 1 - (y / IMG_SIZE)) for x, y in corners]

                                # # Draw the bounding box on the image
                                # for i in range(4):
                                #     cv2.line(
                                #         pc_img,
                                #         (int(corners[i][0]), int(corners[i][1])),
                                #         (int(corners[(i + 1) % 4][0]), int(corners[(i + 1) % 4][1])),
                                #         (255,),
                                #         1,
                                #     )

                                # YOLO format: class_id cx cy w h x1 y1 x2 y2 x3 y3 x4 y4
                                formatted_line = (
                                    f"{class_id} {norm_bbox_centre_px_x:.6f} {norm_bbox_centre_px_y:.6f} {norm_bbox_width:.6f} {norm_bbox_height:.6f} "
                                    + " ".join([f"{x:.6f} {y:.6f}" for x, y in norm_corners])
                                    + "\n"
                                )

                                formatted_lines.append(formatted_line)

                            # Determine the save path for the labels
                            if folder == "testing":
                                label_save_path = Path(
                                    yolo_dir, pointcloud_folder.name, "labels", "val", sample_points.stem + ".txt"
                                )
                            elif folder == "training":
                                label_save_path = Path(
                                    yolo_dir, pointcloud_folder.name, "labels", "train", sample_points.stem + ".txt"
                                )

                            label_save_path.parent.mkdir(parents=True, exist_ok=True)
                            with open(label_save_path, "w") as f:
                                f.writelines(formatted_lines)
                            self.get_logger().info(f"Labels saved at {label_save_path}")

                        # Save imgs
                        if folder == "testing":
                            # Save in val
                            save_path = Path(
                                yolo_dir, pointcloud_folder.name, "images", "val", sample_points.stem + ".jpg"
                            )
                        elif folder == "training":
                            # Save in train
                            save_path = Path(
                                yolo_dir, pointcloud_folder.name, "images", "train", sample_points.stem + ".jpg"
                            )

                        save_path.parent.mkdir(parents=True, exist_ok=True)
                        cv2.imwrite(str(save_path), pc_img)
                        self.get_logger().info(f"Image saved at {save_path}")

    def calculate_bbox_corners(self, cx, cy, width, height, yaw, scale):
        """
        Calculate the four corners of the bounding box given the center, width, height, and yaw angle.

        :param cx: Center x-coordinate.
        :param cy: Center y-coordinate.
        :param width: Width of the bounding box.
        :param height: Height of the bounding box.
        :param yaw: Yaw angle of the bounding box.
        :param scale: Scale factor to convert world coordinates to pixel coordinates.
        :return: List of four corners (x, y) in pixel coordinates.
        """
        corners = []
        half_width = width / 2
        half_height = height / 2

        # Calculate the corners in the local frame
        local_corners = [
            (-half_width, -half_height),
            (half_width, -half_height),
            (half_width, half_height),
            (-half_width, half_height),
        ]

        # Rotate and translate the corners to the global frame
        for x, y in local_corners:
            x_rot = cx + (x * np.cos(yaw) - y * np.sin(yaw))
            y_rot = cy + (x * np.sin(yaw) + y * np.cos(yaw))
            x_pix = int((x_rot + BOUNDS) * scale)
            y_pix = int((y_rot + BOUNDS) * scale)
            corners.append((x_pix, y_pix))

        return corners

    def generate_point_cloud_image(
        self, points: np.ndarray, img_size: int = 1000, bounds: float = 5
    ) -> tuple[np.ndarray, float]:
        """
        Converts a 3D point cloud into a 2D grayscale image.

        :param points: List of (x, y, z) coordinates.
        :param img_size: Size of the output image in pixels (default: 1000x1000).
        :param bounds: Defines the real-world range in meters (default: ±5m).
        :return: NumPy image array of shape (img_size, img_size, 1) and the scale factor.
        """

        # Convert ±bounds meters into img_size pixels
        scale = img_size / (2 * bounds)
        image = np.zeros((img_size, img_size, 1), dtype=np.uint8)  # Single channel

        # Convert 3D points to 2D image coordinates
        for x, y, _ in points:
            if -bounds <= x <= bounds and -bounds <= y <= bounds:
                # Shift and scale x and y
                px = int((x + bounds) * scale)
                py = int((y + bounds) * scale)
                # Ensure the pixel coordinates are within the image bounds
                if 0 <= px < img_size and 0 <= py < img_size:
                    # Make white pixel for each point
                    image[py, px] = 255

        return image, scale

    def apply_mask(self, image: np.ndarray, ratio: int) -> np.ndarray:
        """
        Applies a mask to the image to remove points of the robot itself.

        :param image: Input image.
        :return: Masked image.
        """
        mask_size = IMG_SIZE // ratio  # Size of the mask box (1/ratio of the image size)
        center_x, center_y = image.shape[1] // 2, image.shape[0] // 2

        # Apply the mask
        image[
            center_y - mask_size // 2 : center_y + mask_size // 2, center_x - mask_size // 2 : center_x + mask_size // 2
        ] = 0

        return image


def main(args=None):
    rclpy.init(args=args)

    node = YoloFormat()
    # rclpy.spin(node)
    node.generate_yolo_data()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
