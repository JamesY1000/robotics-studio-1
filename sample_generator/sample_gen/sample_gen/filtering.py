from typing import List, Tuple

import numpy as np

from sample_gen.box_np_ops import points_in_rbbox
from sample_gen.geometry import rotate_points


def remove_occluded_objects(
    points: np.ndarray,
    bboxes: np.ndarray,
    max_occlusion: np.ndarray | list[float] = np.array([0.9, 1.0, 0.7]),
    min_points_object: int = 5,
) -> Tuple[np.ndarray, np.ndarray, List]:
    """Check for occluded objects, according to max_occlusion and return masks of boxes to remove and points inside those boxes.
    Args:
        points: [num_points, 3] array.
        bboxes: [num_boxes, 7] array. (x, y, z, l, w, h, yaw)
        max_occlusion: [x, y, z] array. Remove objects which are occluded more than the given fractions along x, y, z axis
        min_points_object: (int) How many points an object shall at least have to be kept
    Returns:
        remove_points_mask, keep_boxes_mask, occlusions
    """
    keep_boxes_mask = np.zeros((bboxes.shape[0]), dtype=np.bool_)
    remove_points_mask = np.zeros((points.shape[0]), dtype=np.bool_)

    mask = points_in_rbbox(points, bboxes)
    occlusions = []

    for i in range(bboxes.shape[0]):
        points_inside = points[mask[:, i], :]
        # Check for any points first
        if points_inside.shape[0] > 0:
            points_rot = rotate_points(points_inside, bboxes[i, 6])

            distances = np.amax(points_rot, axis=0) - np.amin(points_rot, axis=0)
            occlusion = 1 - (distances / bboxes[i, 3:6])
            # 0 = fully visible if smaller than 20%
            if occlusion[0] < 0.2:
                occlusions.append(0)
            # 1 = partly occluded if between 20% and 50%
            elif occlusion[0] < 0.5:
                occlusions.append(1)
            # 2 = largely occluded if between 50% and maximum occlusion
            elif occlusion[0] < max_occlusion[0]:
                occlusions.append(2)

            # Drop boxes which are more occluded than max
            if np.any(occlusion > max_occlusion) or points_rot.shape[0] < min_points_object:
                keep_boxes_mask[i] = False
                remove_points_mask = remove_points_mask | mask[:, i]
            else:
                keep_boxes_mask[i] = True

        else:
            keep_boxes_mask[i] = False

    return remove_points_mask, keep_boxes_mask, occlusions
