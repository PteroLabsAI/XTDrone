"""PX4's NED world and FRD body against ROS's ENU world and FLU body, and the optical frame ego-planner projects depth in.

Quaternions are (w, x, y, z); q_A_B takes a vector's B components into A's.
"""
import math

_HALF_SQRT2 = math.sqrt(0.5)
# NED -> ENU: x and y swap, z flips; 180 deg about (1, 1, 0) (px4_ros_com frame_transforms.cpp NED_ENU_Q).
Q_ENU_NED = (0.0, _HALF_SQRT2, _HALF_SQRT2, 0.0)
# FLU -> FRD: 180 deg about x (frame_transforms.cpp AIRCRAFT_BASELINK_Q).
Q_FRD_FLU = (0.0, 1.0, 0.0, 0.0)
# The optical frame (x image right, y image down, z forward) in FLU: ego-planner's cam2body_ (grid_map.cpp:131).
Q_FLU_OPTICAL = (0.5, -0.5, 0.5, -0.5)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def rotate(q, v):
    _, x, y, z = qmul(qmul(q, (0.0, *v)), (q[0], -q[1], -q[2], -q[3]))
    return (x, y, z)


def ned_enu(v):
    """NED components to ENU, and ENU to NED: the swap is its own inverse."""
    return (v[1], v[0], -v[2])


def frd_to_flu(v):
    return (v[0], -v[1], -v[2])


def body_q_enu_flu(q_ned_frd):
    """PX4's attitude (FRD body into NED) as ROS's (FLU body into ENU)."""
    return qmul(qmul(Q_ENU_NED, q_ned_frd), Q_FRD_FLU)


def yaw_ned(q_ned_frd):
    """Heading of an FRD body in NED, radians from north towards east (PX4's Eulerf yaw)."""
    w, x, y, z = q_ned_frd
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_enu_to_ned(yaw):
    """ENU yaw (from east, counter-clockwise) to NED yaw (from north, clockwise): pi/2 - yaw, wrapped to [-pi, pi]."""
    return math.atan2(math.cos(yaw), math.sin(yaw))
