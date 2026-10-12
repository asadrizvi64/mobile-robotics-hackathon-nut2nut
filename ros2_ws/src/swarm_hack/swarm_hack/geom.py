"""Small geometry helpers (no tf_transformations / transforms3d dependency)."""
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def se2_apply(T, p):
    """Point p expressed in frame B -> frame A, where T = (tx, ty, th) is B's pose in A."""
    tx, ty, th = T
    c, s = math.cos(th), math.sin(th)
    return tx + c * p[0] - s * p[1], ty + s * p[0] + c * p[1]


def se2_inverse_apply(T, p):
    """Point p expressed in frame A -> frame B (inverse of se2_apply)."""
    tx, ty, th = T
    c, s = math.cos(th), math.sin(th)
    dx, dy = p[0] - tx, p[1] - ty
    return c * dx + s * dy, -s * dx + c * dy


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_to_matrix(tf):
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix (maps child coords into parent)."""
    t, q = tf.transform.translation, tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T
