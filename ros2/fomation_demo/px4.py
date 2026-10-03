"""PX4's own ROS 2 interface: the uXRCE-DDS topics its client publishes and reads for one vehicle."""
from rclpy.qos import qos_profile_sensor_data

# PX4's client writes best effort (uxrce_dds_client/utilities.hpp), so a reliable subscription would match nothing.
QOS = qos_profile_sensor_data


def px4_topic(vehicle, direction, name, msg_type):
    """/<vehicle>/fmu/<in|out>/<name>, with _v<N> for a versioned message, the name PX4's client gives it."""
    version = getattr(msg_type, "MESSAGE_VERSION", 0)
    return "/%s/fmu/%s/%s%s" % (vehicle, direction, name, "_v%d" % version if version else "")
