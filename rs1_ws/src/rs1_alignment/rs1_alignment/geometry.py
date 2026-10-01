"""SE(2) operations, deliberately independent of ROS and vertical estimation."""
from math import atan2, cos, sin


def wrap(angle):
    return atan2(sin(angle), cos(angle))


def yaw(q):
    return atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def compose(a, b):
    x, y, theta = a
    bx, by, bt = b
    return (x + cos(theta)*bx - sin(theta)*by,
            y + sin(theta)*bx + cos(theta)*by, wrap(theta+bt))


def registration(world_base, map_base):
    """world_T_map = world_T_base * inverse(map_T_base)."""
    wx, wy, wt = world_base
    mx, my, mt = map_base
    theta = wrap(wt-mt)
    return wx-cos(theta)*mx+sin(theta)*my, wy-sin(theta)*mx-cos(theta)*my, theta
