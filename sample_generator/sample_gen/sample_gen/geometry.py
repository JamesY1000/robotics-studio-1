import numba
import numpy as np
import tf_transformations
from geometry_msgs.msg import TransformStamped

from .box_np_ops import corner_to_standup_nd_jit


@numba.jit
def do_transform_jit(points: np.ndarray, rot_matrix: np.ndarray, translation: np.ndarray) -> np.ndarray:
    points = points @ rot_matrix
    points = points[:, :] + translation
    return points


@numba.jit
def do_rotate_jit(points: np.ndarray, rot_matrix: np.ndarray) -> np.ndarray:
    points = points @ rot_matrix
    return points


def rotate_points(points: np.ndarray, yaw: float) -> np.ndarray:
    quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
    rot_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3].astype("float32")
    points = points.astype("float32")
    return do_rotate_jit(points, rot_matrix)


def transform_points(points: np.ndarray, trafo: TransformStamped) -> np.ndarray:
    transl = trafo.transform.translation
    quat = trafo.transform.rotation
    quat = [quat.x, quat.y, quat.z, -quat.w]  # Inverse quaternion by negating w comp
    translation = np.array([transl.x, transl.y, transl.z], dtype="float32")
    rot_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3].astype("float32")
    points = points.astype("float32")
    return do_transform_jit(points, rot_matrix, translation)


@numba.jit
def points_in_2d_polygon_jit(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Check points is in 2d convex polygons. True when point in polygon.

    Args:
        points (np.ndarray): Input points with the shape of [num_points, 2].
        polygon (np.ndarray): Input polygon with the shape of [num_points_of_polygon, 2].

    Returns:
        np.ndarray: Result if points are inside.
    """
    numPoints = points.shape[0]
    numEdges = polygon.shape[0]
    ret = np.zeros((numPoints), dtype=np.bool_)
    for idxPoint in range(points.shape[0]):
        pointX = points[idxPoint, 0]
        pointY = points[idxPoint, 1]
        inside = False
        idxEdge2 = numEdges - 1
        idxEdge1 = 0
        edgesX = polygon[:, 0]
        edgesY = polygon[:, 1]
        for _ in range(numEdges):
            if ((edgesY[idxEdge1] > pointY) != (edgesY[idxEdge2] > pointY)) and (
                pointX
                < (edgesX[idxEdge2] - edgesX[idxEdge1])
                * (pointY - edgesY[idxEdge1])
                / (edgesY[idxEdge2] - edgesY[idxEdge1])
                + edgesX[idxEdge1]
            ):
                inside = not inside

            idxEdge2 = idxEdge1
            idxEdge1 += 1

        ret[idxPoint] = inside
    return ret


@numba.njit
def box_collision_test(boxes: np.ndarray, qboxes: np.ndarray, clockwise: bool = True) -> np.ndarray:
    N = boxes.shape[0]
    K = qboxes.shape[0]
    ret = np.zeros((N, K), dtype=np.bool_)
    slices = np.array([1, 2, 3, 0])
    lines_boxes = np.stack((boxes, boxes[:, slices, :]), axis=2)  # [N, 4, 2(line), 2(xy)]
    lines_qboxes = np.stack((qboxes, qboxes[:, slices, :]), axis=2)
    # vec = np.zeros((2,), dtype=boxes.dtype)
    boxes_standup = corner_to_standup_nd_jit(boxes)
    qboxes_standup = corner_to_standup_nd_jit(qboxes)
    for i in range(N):
        for j in range(K):
            # calculate standup first
            iw = min(boxes_standup[i, 2], qboxes_standup[j, 2]) - max(boxes_standup[i, 0], qboxes_standup[j, 0])
            if iw > 0:
                ih = min(boxes_standup[i, 3], qboxes_standup[j, 3]) - max(boxes_standup[i, 1], qboxes_standup[j, 1])
                if ih > 0:
                    for k in range(4):
                        for m in range(4):
                            A = lines_boxes[i, k, 0]
                            B = lines_boxes[i, k, 1]
                            C = lines_qboxes[j, m, 0]
                            D = lines_qboxes[j, m, 1]
                            acd = (D[1] - A[1]) * (C[0] - A[0]) > (C[1] - A[1]) * (D[0] - A[0])
                            bcd = (D[1] - B[1]) * (C[0] - B[0]) > (C[1] - B[1]) * (D[0] - B[0])
                            if acd != bcd:
                                abc = (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
                                abd = (D[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (D[0] - A[0])
                                if abc != abd:
                                    ret[i, j] = True  # collision.
                                    break
                        if ret[i, j] is True:
                            break
                    if ret[i, j] is False:
                        # now check complete overlap.
                        # box overlap qbox:
                        box_overlap_qbox = True
                        for m in range(4):  # point l in qboxes
                            for k in range(4):  # corner k in boxes
                                vec = boxes[i, k] - boxes[i, (k + 1) % 4]
                                if clockwise:
                                    vec = -vec
                                cross = vec[1] * (boxes[i, k, 0] - qboxes[j, m, 0])
                                cross -= vec[0] * (boxes[i, k, 1] - qboxes[j, m, 1])
                                if cross >= 0:
                                    box_overlap_qbox = False
                                    break
                            if box_overlap_qbox is False:
                                break

                        if box_overlap_qbox is False:
                            qbox_overlap_box = True
                            for m in range(4):  # point l in boxes
                                for k in range(4):  # corner k in qboxes
                                    vec = qboxes[j, k] - qboxes[j, (k + 1) % 4]
                                    if clockwise:
                                        vec = -vec
                                    cross = vec[1] * (qboxes[j, k, 0] - boxes[i, m, 0])
                                    cross -= vec[0] * (qboxes[j, k, 1] - boxes[i, m, 1])
                                    if cross >= 0:  #
                                        qbox_overlap_box = False
                                        break
                                if qbox_overlap_box is False:
                                    break
                            if qbox_overlap_box:
                                ret[i, j] = True  # collision.
                        else:
                            ret[i, j] = True  # collision.
    return ret
