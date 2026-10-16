import math
import socket

import numpy as np
import pytest

from pterosim_ego import stamp_from_us
from pterosim_ego.depth_bridge import (PTF1, PTF1_DEPTH_F32_METRES, PTF1_MAGIC, depth_to_mm, frame_error, read_frame,
                                       to_image)
from pterosim_ego.frames import Q_FLU_OPTICAL, body_q_enu_flu, ned_enu, qmul, rotate, yaw_enu_to_ned, yaw_ned

TOL = 1e-9


def close(a, b):
    return all(abs(x - y) < TOL for x, y in zip(a, b))


def heading_q(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def test_level_drone_facing_north_faces_plus_y_in_enu():
    q = body_q_enu_flu(heading_q(0.0))
    assert close(rotate(q, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))  # nose: north
    assert close(rotate(q, (0.0, 1.0, 0.0)), (-1.0, 0.0, 0.0))  # left wing: west
    assert close(rotate(q, (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0))  # up stays up


def test_drone_facing_east_and_yaw_round_trip():
    q = heading_q(math.pi / 2)
    assert close(rotate(body_q_enu_flu(q), (1.0, 0.0, 0.0)), (1.0, 0.0, 0.0))
    assert abs(yaw_ned(q) - math.pi / 2) < TOL
    assert abs(yaw_enu_to_ned(0.0) - math.pi / 2) < TOL  # ENU east is NED 90 deg
    assert abs(yaw_enu_to_ned(math.pi) - (-math.pi / 2)) < TOL  # west, wrapped


def test_ned_enu_swaps_both_ways():
    assert ned_enu((1.0, 2.0, 3.0)) == (2.0, 1.0, -3.0)
    assert ned_enu(ned_enu((1.0, 2.0, 3.0))) == (1.0, 2.0, 3.0)


def test_optical_axes_in_flu():
    assert close(rotate(Q_FLU_OPTICAL, (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))  # looks forward
    assert close(rotate(Q_FLU_OPTICAL, (1.0, 0.0, 0.0)), (0.0, -1.0, 0.0))  # image right: body right
    assert close(rotate(Q_FLU_OPTICAL, (0.0, 1.0, 0.0)), (0.0, 0.0, -1.0))  # image down: body down
    camera = qmul(body_q_enu_flu(heading_q(0.0)), Q_FLU_OPTICAL)
    assert close(rotate(camera, (0.0, 0.0, 1.0)), (0.0, 1.0, 0.0))  # drone facing north sees north


def test_depth_to_mm():
    metres = np.array([[math.inf, math.nan, 0.5, 70.0, -1.0, 0.0004, 65.535]], dtype="<f4")
    assert depth_to_mm(metres).tolist() == [[0, 0, 500, 0, 0, 0, 65535]]
    assert depth_to_mm(metres).dtype == np.uint16


def test_stamp_from_us():
    stamp = stamp_from_us(12_345_678)
    assert (stamp.sec, stamp.nanosec) == (12, 345_678_000)


def ptf1_bytes(width, height, sim_us, intrinsics, position, rotation, pixels, extra=b""):
    payload = pixels.astype("<f4").tobytes()
    header = PTF1.pack(PTF1_MAGIC, PTF1.size + len(extra), PTF1_DEPTH_F32_METRES, width, height, width * 4, 7,
                       sim_us, 0, *intrinsics, *position, *rotation, len(payload))
    return header + extra + payload


def test_read_frame_and_checks():
    width, height = 4, 2
    intrinsics = (2.0, 2.0, 1.5, 0.5)
    pixels = np.arange(width * height, dtype="<f4").reshape(height, width)
    a, b = socket.socketpair()
    with a, b:
        a.sendall(ptf1_bytes(width, height, 42, intrinsics, (0.15, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), pixels,
                             extra=b"\0" * 8))
        frame = read_frame(b)
    assert (frame.width, frame.height, frame.sim_us) == (width, height, 42)
    assert np.array_equal(frame.pixels, pixels)
    image = to_image(frame, "cam")
    assert (image.step, len(image.data), image.header.stamp.nanosec) == (width * 2, width * height * 2, 42_000)
    assert np.array_equal(np.frombuffer(image.data, dtype="<u2").reshape(height, width), depth_to_mm(pixels))
    assert frame_error(frame, intrinsics, (0.15, 0.0, 0.0)) is None
    assert "fx, fy, cx, cy" in frame_error(frame, (2.5, 2.0, 1.5, 0.5), (0.15, 0.0, 0.0))
    assert "camera_offset_frd" in frame_error(frame, intrinsics, (0.0, 0.0, 0.0))
    assert "level forward" in frame_error(frame._replace(rotation_frd=(0.99, 0.0, 0.1, 0.0)), intrinsics,
                                          (0.15, 0.0, 0.0))
    assert "640x480" in frame_error(frame._replace(width=1280, height=720), intrinsics, (0.15, 0.0, 0.0))


def test_read_frame_refuses_a_colour_stream():
    a, b = socket.socketpair()
    with a, b:
        header = bytearray(ptf1_bytes(1, 1, 0, (1.0,) * 4, (0.0,) * 3, (1.0, 0.0, 0.0, 0.0), np.zeros((1, 1))))
        header[8:12] = (1).to_bytes(4, "little")  # BGRA8
        a.sendall(header)
        with pytest.raises(ValueError, match="not depth"):
            read_frame(b)
