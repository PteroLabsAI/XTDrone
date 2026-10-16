"""One drone's PteroSim depth camera into ego-planner: PTF1 frames over TCP to a 16UC1 millimetre Image.

The image is stamped with the frame's sim_us, the simulation time of the state it shows (pterosim_clock publishes the
same clock). Anything wrong with the stream, or a frame that disagrees with the intrinsics or the camera mount the
planner and px4_bridge were given, ends the node.
"""
import array
import socket
import struct
from typing import NamedTuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from pterosim_ego import stamp_from_us

# PTF1 as the SDK reads it (PteroSimScripting SDK/python/pterosim/streams.py:14): magic, header_bytes, format, width,
# height, row_bytes, frame, sim_us, capture_unix_us, fx, fy, cx, cy, camera position (m, body FRD), rotation (w, x, y, z;
# camera X optical axis, Y image right, Z image down, into body FRD), payload_bytes. Little-endian.
PTF1 = struct.Struct("<4sIIIII" "QQQ" "dddd" "ddd" "dddd" "Q")
PTF1_MAGIC = b"PTF1"
PTF1_DEPTH_F32_METRES = 2  # PteroSimSensors SensorFrame.h ESensorFrameFormat::DepthF32Metres
RAW_STREAM_PORT_STRIDE = 2  # a sensor serves raw_stream_port + 2 * instance id (BaseRenderSensor.h:188)
MM_PER_M = 1000.0  # ego-planner's k_depth_scaling_factor (advanced_param.launch.py:162)
UINT16_MAX = 65535
# grid_map.cpp:128 sizes its projection buffer for 640x480 and never checks the image it is given.
MAX_PIXELS = 640 * 480
INTRINSICS_TOLERANCE_PX = 0.01  # chosen: the sensor's float FOV moves fx by ~1e-5 px
MOUNT_TOLERANCE = 1e-3  # metres, and quaternion components (~0.1 deg); chosen, far under a 0.1 m voxel
IMAGE_QUEUE = 2  # chosen: one frame in flight, one waiting


class Frame(NamedTuple):
    fmt: int
    width: int
    height: int
    sim_us: int
    intrinsics: tuple  # fx, fy, cx, cy
    position_frd: tuple
    rotation_frd: tuple  # w, x, y, z
    pixels: np.ndarray


def recv_exact(sock, count):
    data = bytearray(count)
    view = memoryview(data)
    got = 0
    while got < count:
        n = sock.recv_into(view[got:])
        if n == 0:
            raise ConnectionError(f"raw stream closed by the simulator after {got} of {count} bytes")
        got += n
    return data


def read_frame(sock):
    """The next PTF1 frame on sock; the parsing of the SDK's RawStream.read (streams.py:42-57)."""
    (magic, header_bytes, fmt, width, height, row_bytes, _frame, sim_us, _capture_unix_us,
     fx, fy, cx, cy, px, py, pz, qw, qx, qy, qz, payload_bytes) = PTF1.unpack(recv_exact(sock, PTF1.size))
    if magic != PTF1_MAGIC:
        raise ValueError(f"not a PTF1 frame: magic {magic!r}")
    if header_bytes < PTF1.size:
        raise ValueError(f"PTF1 header of {header_bytes} bytes, shorter than the {PTF1.size} it must hold")
    recv_exact(sock, header_bytes - PTF1.size)  # fields a newer simulator appended
    if fmt != PTF1_DEPTH_F32_METRES:
        raise ValueError(f"PTF1 format {fmt}, not depth ({PTF1_DEPTH_F32_METRES}): is raw_stream_port a depth camera's?")
    want_row = width * np.dtype("<f4").itemsize
    if row_bytes != want_row or payload_bytes != want_row * height:
        raise ValueError(f"PTF1 {width}x{height} depth: rows of {row_bytes} bytes and {payload_bytes} in all, "
                         f"where it takes {want_row} and {want_row * height}")
    pixels = np.frombuffer(recv_exact(sock, payload_bytes), dtype="<f4").reshape(height, width)
    return Frame(fmt, width, height, sim_us, (fx, fy, cx, cy), (px, py, pz), (qw, qx, qy, qz), pixels)


def depth_to_mm(metres):
    """float32 metres to uint16 millimetres; no hit (inf), nan, <= 0 or past 16 bits give 0, which ego-planner reads as
    free out to max_ray_length (grid_map.cpp:327)."""
    with np.errstate(invalid="ignore", over="ignore"):
        mm = np.rint(metres * MM_PER_M)
        valid = (mm > 0) & (mm <= UINT16_MAX)
    return np.where(valid, mm, 0).astype(np.uint16)


def to_image(frame, frame_id):
    """The frame as ego-planner's 16UC1 millimetre Image, stamped with its simulation time."""
    image = Image(height=frame.height, width=frame.width, encoding="16UC1", is_bigendian=0,
                  step=frame.width * np.dtype(np.uint16).itemsize)
    image.header.stamp = stamp_from_us(frame.sim_us)
    image.header.frame_id = frame_id
    # Humble's Image.data takes an array('B') as is, anything else element by element; frombytes wants a byte view.
    data = array.array("B")
    data.frombytes(depth_to_mm(frame.pixels).view(np.uint8))
    image.data = data
    return image


def frame_error(frame, intrinsics, camera_offset_frd):
    """Why the planner cannot use this frame as configured, or None."""
    if frame.width * frame.height > MAX_PIXELS:
        return f"{frame.width}x{frame.height} is over ego-planner's 640x480 buffer (grid_map.cpp:128)"
    if any(abs(a - b) > INTRINSICS_TOLERANCE_PX for a, b in zip(frame.intrinsics, intrinsics)):
        return f"frame fx, fy, cx, cy {frame.intrinsics} != the planner's {tuple(intrinsics)}: check width, height, fov"
    if any(abs(a - b) > MOUNT_TOLERANCE for a, b in zip(frame.position_frd, camera_offset_frd)):
        return f"camera at {frame.position_frd} m in body FRD, camera_offset_frd says {tuple(camera_offset_frd)}"
    if any(abs(c) > MOUNT_TOLERANCE for c in frame.rotation_frd[1:]):
        return f"camera rotated {frame.rotation_frd} (w, x, y, z) in the body; only a level forward mount is supported"
    return None


class DepthBridge(Node):
    def __init__(self):
        super().__init__("depth_bridge")
        for name, kind in (("drone_id", rclpy.Parameter.Type.INTEGER), ("host", rclpy.Parameter.Type.STRING),
                           ("raw_stream_port", rclpy.Parameter.Type.INTEGER), ("fx", rclpy.Parameter.Type.DOUBLE),
                           ("fy", rclpy.Parameter.Type.DOUBLE), ("cx", rclpy.Parameter.Type.DOUBLE),
                           ("cy", rclpy.Parameter.Type.DOUBLE),
                           ("camera_offset_frd", rclpy.Parameter.Type.DOUBLE_ARRAY)):
            self.declare_parameter(name, kind)
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        self.drone_id = value("drone_id")
        self.host = value("host")
        self.port = value("raw_stream_port") + RAW_STREAM_PORT_STRIDE * self.drone_id
        self.intrinsics = (value("fx"), value("fy"), value("cx"), value("cy"))
        self.camera_offset_frd = tuple(value("camera_offset_frd"))
        self.frame_id = f"drone_{self.drone_id}_depth_optical"
        self.image_pub = self.create_publisher(Image, "depth", IMAGE_QUEUE)

    def run(self):
        sock = socket.create_connection((self.host, self.port))
        self.get_logger().info(f"connected to {self.host}:{self.port}, publishing on {self.image_pub.topic_name}")
        published = 0
        while rclpy.ok():
            frame = read_frame(sock)
            error = frame_error(frame, self.intrinsics, self.camera_offset_frd)
            if error:
                raise ValueError(f"{self.host}:{self.port}: {error}")
            image = to_image(frame, self.frame_id)
            self.image_pub.publish(image)
            if published == 0:
                self.get_logger().info(f"first frame: {frame.width}x{frame.height} at sim time {frame.sim_us} us")
            published += 1


def main():
    # Ctrl+C as KeyboardInterrupt: rclpy's own handler would shut the context while recv blocks, and the next publish
    # would fail instead.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DepthBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
