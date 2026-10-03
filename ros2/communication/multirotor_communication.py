"""ROS 2 port of communication/multirotor_communication.py: the velocity and command paths the formation uses, on PX4's
own ROS 2 interface (px4_msgs over the uXRCE-DDS agent).

    python3 multirotor_communication.py <vehicle_type> <vehicle_id> [<vehicle_id> ...]

Every vehicle named is served by one node in one process; a process per vehicle cost the 18-vehicle fleet two cores.
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.duration import Duration
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fomation_demo"))
from px4 import QOS, px4_topic  # noqa: E402

SETPOINT_HZ = 30.0  # the ROS 1 node's rospy.Rate(30)
HOVER_LIN = 0.02  # m/s, the ROS 1 node's hover_state_transition
HOVER_ANG = 0.005  # rad/s, same
# A velocity nobody repeats for this long is dropped for a hover, so a dead sender cannot fly the vehicle away;
# chosen: well over every sender's period (keyboard 0.1 s, followers 1/30 s), half PX4's COM_OF_LOSS_T default.
VELOCITY_TIMEOUT_S = 0.5
STOPPED_MS = 0.1  # below this a hover takes the point it is at; chosen, no evidence
NAN3 = [math.nan] * 3
# MAVROS read the ROS 1 node's all-zero first target as: hold the estimator's origin facing east (NED yaw pi/2).
START_TARGET = ("position", (0.0, 0.0, 0.0), math.pi / 2)
# VEHICLE_CMD_DO_SET_MODE with MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4 main and sub mode (commander/px4_custom_mode.h)
CUSTOM_MODE_ENABLED = 1.0
MODES = {"OFFBOARD": (6, 0), "AUTO.TAKEOFF": (4, 2), "AUTO.RTL": (4, 5), "AUTO.LAND": (4, 6)}
# DDS from PX4 is best effort: a command is sent again until PX4 acknowledges it, as MAVLink's command protocol does.
COMMAND_RETRY_S = 0.5  # chosen: tens of PX4 cycles, well under a person's patience at the keyboard
COMMAND_ATTEMPTS = 5  # chosen; measured at 18 vehicles: 1 of 54 commands lost, 3 of 54 needing a second attempt
COMMANDER_COMPONENT = 1  # MAV_COMP_ID_AUTOPILOT1, commander's component id
# A source below COMPONENT_MODE_EXECUTOR_START makes a mode change the user's (commander/Commander.cpp)
SOURCE_SYSTEM = SOURCE_COMPONENT = 1
RESULT_PREFIX = "VEHICLE_CMD_RESULT_"
RESULTS = {getattr(VehicleCommandAck, n): n[len(RESULT_PREFIX):] for n in dir(VehicleCommandAck) if n.startswith(RESULT_PREFIX)}
NO_POSITION_WARN_S = 5.0  # chosen: PX4 takes about 20 s to boot, a few lines per vehicle


class Communication:
    """One vehicle's bridge on a shared node; its log lines carry the vehicle's name."""

    def __init__(self, node, vehicle_type, vehicle_id):
        name = vehicle_type + '_' + vehicle_id
        self.node = node
        self.log = node.get_logger().get_child(name)
        self.system_id = int(vehicle_id) + 1  # rcS: MAV_SYS_ID = instance + 1
        self.position = None  # PX4's latest VehicleLocalPosition; its timestamp, PX4's own clock, stamps every message here
        self.position_topic = px4_topic(name, "out", "vehicle_local_position", VehicleLocalPosition)
        self.nav_state = None
        self.asked_mode = None  # the last mode this bridge asked PX4 for
        self.flight_mode = None
        self.velocity_at = None
        self.target = START_TARGET
        self.pending = {}  # command id -> [VehicleCommand, when it was last sent, attempts] until PX4 acknowledges it

        node.create_subscription(VehicleLocalPosition, self.position_topic, lambda m: setattr(self, "position", m), QOS)
        node.create_subscription(VehicleStatus, px4_topic(name, "out", "vehicle_status", VehicleStatus),
                                 lambda m: setattr(self, "nav_state", m.nav_state), QOS)
        node.create_subscription(VehicleCommandAck, px4_topic(name, "out", "vehicle_command_ack", VehicleCommandAck),
                                 self.ack_callback, QOS)
        node.create_subscription(Twist, "/xtdrone/" + name + "/cmd_vel_flu", lambda m: self.velocity(m, "flu"), 1)
        node.create_subscription(Twist, "/xtdrone/" + name + "/cmd_vel_enu", lambda m: self.velocity(m, "enu"), 1)
        node.create_subscription(String, "/xtdrone/" + name + "/cmd", self.cmd_callback, 3)
        self.mode_pub = node.create_publisher(OffboardControlMode, px4_topic(name, "in", "offboard_control_mode", OffboardControlMode), QOS)
        self.setpoint_pub = node.create_publisher(TrajectorySetpoint, px4_topic(name, "in", "trajectory_setpoint", TrajectorySetpoint), QOS)
        self.command_pub = node.create_publisher(VehicleCommand, px4_topic(name, "in", "vehicle_command", VehicleCommand), QOS)
        self.log.info("communication initialized")

    def tick(self):
        if self.position is None:
            # Stamped with PX4's clock, offboard lapses after COM_OF_LOSS_T when this bridge stops hearing PX4.
            self.log.warn("nothing from PX4 on %s yet" % self.position_topic, throttle_duration_sec=NO_POSITION_WARN_S)
            return
        self.resend_pending()
        stale = self.velocity_at is None or self.node.get_clock().now() - self.velocity_at > Duration(seconds=VELOCITY_TIMEOUT_S)
        if self.flight_mode == "OFFBOARD" and stale:
            self.log.warn("no velocity command for %gs, hovering" % VELOCITY_TIMEOUT_S)
            self.hover()
        elif self.flight_mode == "BRAKE":
            self.hover()
        kind = self.target[0]
        stamp = self.position.timestamp
        self.mode_pub.publish(OffboardControlMode(timestamp=stamp, position=kind == "position", velocity=kind == "velocity"))
        # PX4's client takes /fmu/in/trajectory_setpoint in any mode, where it would fight AUTO.LAND; MAVLink's was dropped.
        # Asked for another mode, it stops at once: vehicle_status (5 Hz) would say OFFBOARD for up to 0.2 s more.
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.asked_mode in (None, "OFFBOARD"):
            self.setpoint_pub.publish(self.setpoint(stamp))

    def setpoint(self, stamp):
        if self.target[0] == "position":
            _, position, yaw = self.target
            return TrajectorySetpoint(timestamp=stamp, position=list(position), velocity=NAN3, acceleration=NAN3,
                                      jerk=NAN3, yaw=yaw, yawspeed=math.nan)
        _, frame, (x, y, z), w = self.target
        if frame == "enu":
            north, east = y, x
        else:
            # FLU turned by the current heading, as PX4 turned MAVROS's BODY_NED on every setpoint it received.
            h = self.position.heading
            north, east = math.cos(h) * x + math.sin(h) * y, math.sin(h) * x - math.cos(h) * y
        return TrajectorySetpoint(timestamp=stamp, position=NAN3, velocity=[north, east, -z], acceleration=NAN3,
                                  jerk=NAN3, yaw=math.nan, yawspeed=-w)

    def velocity(self, msg, frame):
        if self.position is None:
            # A sender can start before PX4 has a local position to hover at.
            self.log.warn("no local_position yet, velocity dropped", throttle_duration_sec=2.0)
            return
        v, w = msg.linear, msg.angular.z
        if abs(v.x) > HOVER_LIN or abs(v.y) > HOVER_LIN or abs(v.z) > HOVER_LIN or abs(w) > HOVER_ANG:
            self.flight_mode = "OFFBOARD"
            self.velocity_at = self.node.get_clock().now()
            self.target = ("velocity", frame, (v.x, v.y, v.z), w)
        elif self.flight_mode != "HOVER":
            self.hover()

    def cmd_callback(self, msg):
        # Every command acts, a repeat too: PX4 refuses OFFBOARD or ARM until it is ready, and the retry must get through.
        if msg.data in ('', 'stop controlling'):
            return
        if msg.data in ('ARM', 'DISARM'):
            action = VehicleCommand.ARMING_ACTION_ARM if msg.data == 'ARM' else VehicleCommand.ARMING_ACTION_DISARM
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, float(action))
        elif msg.data == 'HOVER':
            self.hover()
        elif msg.data[:-1] != "mission":
            if msg.data not in MODES:
                self.log.error("unknown mode %r, nothing sent; known: %s" % (msg.data, ", ".join(MODES)))
                return
            # As the ROS 1 node did: the next zero velocity then holds wherever the vehicle is by then.
            self.flight_mode = msg.data
            self.asked_mode = msg.data
            main, sub = MODES[msg.data]
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, CUSTOM_MODE_ENABLED, float(main), float(sub))

    def command(self, command, param1, param2=0.0, param3=0.0):
        if self.position is None:
            self.log.error("command %d dropped: nothing from PX4 on %s yet" % (command, self.position_topic))
            return
        if command in self.pending:
            self.log.warn("command %d replaced before PX4 acknowledged it" % command)
        msg = VehicleCommand(command=command, param1=param1, param2=param2, param3=param3, target_system=self.system_id,
                             target_component=COMMANDER_COMPONENT, source_system=SOURCE_SYSTEM,
                             source_component=SOURCE_COMPONENT, from_external=True)
        self.pending[command] = [msg, None, 0]
        self.resend_pending()

    def resend_pending(self):
        now = self.node.get_clock().now()
        for command, (msg, sent_at, attempts) in list(self.pending.items()):
            if sent_at is not None and now - sent_at < Duration(seconds=COMMAND_RETRY_S):
                continue
            if attempts == COMMAND_ATTEMPTS:
                self.log.error("command %d: no acknowledgement after %d attempts" % (command, attempts))
                del self.pending[command]
                continue
            msg.timestamp = self.position.timestamp
            self.command_pub.publish(msg)
            self.pending[command] = [msg, now, attempts + 1]

    def ack_callback(self, msg):
        sent = self.pending.pop(msg.command, None)
        if sent is None:
            return
        text = "command %d: %s after %d attempt(s)" % (msg.command, RESULTS[msg.result], sent[2])
        # Two call sites: rclpy refuses one site logging at two severities.
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.log.info(text)
        else:
            self.log.warn(text)

    def hover(self):
        if self.position is None:
            self.log.error("HOVER before the first local_position: nowhere to hold yet", throttle_duration_sec=2.0)
            return
        p = self.position
        if math.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz) > STOPPED_MS:
            # Holding the point it is at now, PX4 would overshoot it by the stopping distance and fly back.
            if self.flight_mode != "BRAKE":
                self.flight_mode = "BRAKE"
                self.target = ("velocity", "enu", (0.0, 0.0, 0.0), 0.0)
                self.log.info("BRAKE")
            return
        self.flight_mode = "HOVER"
        self.target = ("position", (p.x, p.y, p.z), p.heading)
        self.log.info("HOVER")


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    vehicle_type, ids = sys.argv[1], sys.argv[2:]
    rclpy.init()
    node = rclpy.create_node(vehicle_type + '_' + '_'.join(ids) + "_communication")
    vehicles = [Communication(node, vehicle_type, i) for i in ids]

    def tick():
        for v in vehicles:
            v.tick()

    node.create_timer(1.0 / SETPOINT_HZ, tick)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
