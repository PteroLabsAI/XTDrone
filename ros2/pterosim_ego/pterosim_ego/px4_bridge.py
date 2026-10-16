"""One x500 between PX4 (uXRCE-DDS; NED world, FRD body) and ego-planner-swarm (ENU world, FLU body).

Out: the drone's odometry and the depth camera's pose, in the swarm's shared world: spawn_enu plus PX4's local frame,
whose origin is where the drone booted. In: ego-planner's position command, flown as a PX4 OFFBOARD trajectory setpoint
once the bridge has put PX4 in OFFBOARD, armed it and lifted it takeoff_height above where it stood. Until a command
comes, and whenever commands stop, the drone holds its last setpoint.

Everything runs on PX4's odometry and PX4's clock, which under lockstep is the simulation's: no wall clock, and the
setpoints stop, so OFFBOARD lapses (COM_OF_LOSS_T), the moment PX4 goes quiet.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleOdometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Empty

from pterosim_ego import US_PER_S, stamp_from_us
from pterosim_ego.frames import Q_FLU_OPTICAL, body_q_enu_flu, frd_to_flu, ned_enu, qmul, rotate, yaw_enu_to_ned, yaw_ned

# PX4's client writes best effort (uxrce_dds_client/utilities.hpp), so a reliable subscription would match nothing.
PX4_QOS = qos_profile_sensor_data
EGO_QUEUE = 10
WORLD_FRAME = "world"  # ego-planner's grid_map/frame_id (advanced_param.launch.py:177)
# Setpoints PX4 must already be getting before it takes OFFBOARD; chosen as PX4's ROS 2 offboard example (10 at 10 Hz).
STREAM_BEFORE_OFFBOARD_S = 1.0
# VEHICLE_CMD_DO_SET_MODE with MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4 main mode OFFBOARD (commander/px4_custom_mode.h)
CUSTOM_MODE_ENABLED = 1.0
PX4_MAIN_MODE_OFFBOARD = 6.0
COMMANDER_COMPONENT = 1  # MAV_COMP_ID_AUTOPILOT1
# A source below COMPONENT_MODE_EXECUTOR_START makes PX4 take the mode change as the operator's (commander/Commander.cpp)
SOURCE_SYSTEM = SOURCE_COMPONENT = 1
# PX4's DDS is best effort: an unanswered command goes again (as XTDrone's ros2 multirotor_communication.py does).
COMMAND_RETRY_S = 0.5
COMMAND_ATTEMPTS = 5
# Right after boot PX4 refuses to arm ("system power unavailable") and needs ~20 s to be ready; chosen pace and bound.
REJECTED_RETRY_S = 2.0
REJECTED_RETRIES = 30
TAKEOFF_REACHED_M = 0.2  # chosen: well inside the launch's 0.71 m obstacle inflation
# traj_server sends at 100 Hz (traj_server.cpp:248); a command older than this is held, not followed. Chosen: half
# PX4's COM_OF_LOSS_T default of 1 s.
COMMAND_TIMEOUT_S = 0.5
# ego-planner's emergency stop drops a moving command to a standstill where the drone was: flown as a position target
# from speed, PX4 overshoots it and flies back through unplanned space. So the bridge brakes and parks where it stops.
EMERGENCY_FROM_MS = 1.0  # chosen: a planned stop slows through every speed between, an emergency one drops in one sample
SETTLED_MS = 0.1  # ego-planner leaves EMERGENCY_STOP for a new plan below this speed (ego_replan_fsm.cpp:621)
# Ego aims the nose 1 s ahead along its weaving path, swinging the cameras. Chosen: 45 still turns the 90 deg depth
# view onto a new heading within 2 s.
YAW_RATE_MAX_RAD_S = math.radians(45.0)
NAN3 = [math.nan] * 3


def speed(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class Px4Bridge(Node):
    def __init__(self):
        super().__init__("px4_bridge")
        for name, kind in (("drone_id", rclpy.Parameter.Type.INTEGER), ("px4_ns", rclpy.Parameter.Type.STRING),
                           ("spawn_enu", rclpy.Parameter.Type.DOUBLE_ARRAY),
                           ("takeoff_height", rclpy.Parameter.Type.DOUBLE),
                           ("camera_offset_frd", rclpy.Parameter.Type.DOUBLE_ARRAY)):
            self.declare_parameter(name, kind)
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        self.drone_id = value("drone_id")
        self.spawn = tuple(value("spawn_enu"))
        camera_offset_frd = tuple(value("camera_offset_frd"))
        if len(self.spawn) != 3 or len(camera_offset_frd) != 3:
            raise ValueError(f"spawn_enu {self.spawn} and camera_offset_frd {camera_offset_frd} must be 3 numbers each")
        self.camera_offset_flu = frd_to_flu(camera_offset_frd)
        self.takeoff_height = value("takeoff_height")
        ns = value("px4_ns")

        def px4_topic(direction, name, msg_type):
            # PX4's client names a versioned message <name>_v<N> (uxrce_dds_client/utilities.hpp generate_topic_name).
            version = getattr(msg_type, "MESSAGE_VERSION", 0)
            return f"/{ns}/fmu/{direction}/{name}" + (f"_v{version}" if version else "")

        self.odometry_topic = px4_topic("out", "vehicle_odometry", VehicleOdometry)
        self.create_subscription(VehicleOdometry, self.odometry_topic, self.on_odometry, PX4_QOS)
        self.create_subscription(VehicleCommandAck, px4_topic("out", "vehicle_command_ack", VehicleCommandAck),
                                 self.on_ack, PX4_QOS)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, px4_topic("in", "offboard_control_mode", OffboardControlMode), PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, px4_topic("in", "trajectory_setpoint", TrajectorySetpoint), PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, px4_topic("in", "vehicle_command", VehicleCommand), PX4_QOS)

        self.create_subscription(PositionCommand, "position_cmd", self.on_position_cmd, EGO_QUEUE)
        self.odom_pub = self.create_publisher(Odometry, "odom", EGO_QUEUE)
        self.camera_pub = self.create_publisher(PoseStamped, "camera_pose", EGO_QUEUE)
        self.ready_pub = self.create_publisher(
            Empty, "takeoff_done", QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.px4_now = None  # PX4's clock, us: the latest odometry's timestamp
        self.state = None  # stream -> offboard -> arm -> takeoff -> fly
        self.stream_from = None
        self.hold = None  # (position ENU, NED yaw) flown when no command is followed
        self.pending = None  # [VehicleCommand, when to send it next (us), sends unanswered] until PX4 accepts it
        self.rejections = 0
        self.cmd = None
        self.cmd_at = None
        self.holding_stale = False
        self.braking = False
        self.parked = None  # (position ENU, NED yaw) where the drone came to rest after an emergency stop
        self.yaw = None  # (NED yaw flown, PX4 us): ego-planner's yaw followed at YAW_RATE_MAX_RAD_S at most
        self.get_logger().info(f"waiting for PX4 on {self.odometry_topic}")

    def on_odometry(self, msg):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            raise RuntimeError(f"{self.odometry_topic}: pose frame {msg.pose_frame}, velocity frame {msg.velocity_frame}; "
                               "this bridge converts NED only")
        # px4_msgs arrays hold numpy float32, which ROS's float64 fields refuse.
        position_ned, q_ned, velocity_ned, rates_frd = (
            tuple(float(v) for v in a) for a in (msg.position, msg.q, msg.velocity, msg.angular_velocity))
        if not all(math.isfinite(v) for v in (*position_ned, *q_ned, *velocity_ned)):
            self.get_logger().warn("PX4 odometry not valid yet", throttle_duration_sec=5.0)
            return
        self.px4_now = msg.timestamp
        position = tuple(s + p for s, p in zip(self.spawn, ned_enu(position_ned)))
        q = body_q_enu_flu(q_ned)
        stamp = stamp_from_us(msg.timestamp_sample)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = WORLD_FRAME
        odom.child_frame_id = f"drone_{self.drone_id}"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
        o = odom.pose.pose.orientation
        o.w, o.x, o.y, o.z = q
        # Linear velocity in the world frame, as ego-planner reads it (ego_replan_fsm.cpp odometryCallback), not the body.
        v = odom.twist.twist.linear
        v.x, v.y, v.z = ned_enu(velocity_ned)
        w = odom.twist.twist.angular
        w.x, w.y, w.z = frd_to_flu(rates_frd)
        self.odom_pub.publish(odom)

        camera = PoseStamped()
        camera.header = odom.header
        p = camera.pose.position
        p.x, p.y, p.z = (a + b for a, b in zip(position, rotate(q, self.camera_offset_flu)))
        o = camera.pose.orientation
        o.w, o.x, o.y, o.z = qmul(q, Q_FLU_OPTICAL)
        self.camera_pub.publish(camera)

        self.step(position, yaw_ned(q_ned), math.sqrt(sum(v * v for v in velocity_ned)))

    def step(self, position, yaw, drone_speed):
        now = self.px4_now
        if self.state is None:
            self.get_logger().info(f"PX4 is up; streaming setpoints for {STREAM_BEFORE_OFFBOARD_S} s before OFFBOARD")
            self.state, self.stream_from, self.hold = "stream", now, (position, yaw)
        if self.state == "stream" and now - self.stream_from >= STREAM_BEFORE_OFFBOARD_S * US_PER_S:
            self.state = "offboard"
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, CUSTOM_MODE_ENABLED, PX4_MAIN_MODE_OFFBOARD)
        self.resend()
        if self.state == "takeoff" and math.dist(position, self.hold[0]) < TAKEOFF_REACHED_M:
            self.state = "fly"
            self.ready_pub.publish(Empty())
            self.get_logger().info(f"takeoff done at {tuple(round(c, 2) for c in position)}; following ego-planner")

        if self.state == "fly" and self.cmd is not None:
            c = self.cmd
            target = (c.position.x, c.position.y, c.position.z)
            if now - self.cmd_at <= COMMAND_TIMEOUT_S * US_PER_S:
                self.holding_stale = False
                if self.braking and speed(c.velocity) == 0.0:
                    if self.parked is None and drone_speed > SETTLED_MS:
                        self.publish_brake(yaw)
                        return
                    if self.parked is None:
                        self.parked = self.hold = (position, yaw)
                        self.get_logger().info(f"parked at {tuple(round(p, 2) for p in position)} after the emergency stop")
                    self.publish_setpoint(*self.parked)
                    return
                self.braking, self.parked = False, None
                flown, rate = self.follow_yaw(yaw_enu_to_ned(c.yaw), yaw, now)
                self.hold = (target, flown)
                self.publish_setpoint(target, flown, (c.velocity.x, c.velocity.y, c.velocity.z),
                                      (c.acceleration.x, c.acceleration.y, c.acceleration.z), rate)
                return
            if not self.holding_stale:
                self.holding_stale = True
                self.get_logger().warn(f"no position command for {COMMAND_TIMEOUT_S} s; holding the last one")
        self.publish_setpoint(*self.hold)

    def publish_setpoint(self, position, yaw, velocity=None, acceleration=None, yawspeed=math.nan):
        local = [p - s for p, s in zip(position, self.spawn)]
        self.mode_pub.publish(OffboardControlMode(timestamp=self.px4_now, position=True))
        self.setpoint_pub.publish(TrajectorySetpoint(
            timestamp=self.px4_now, position=list(ned_enu(local)),
            velocity=list(ned_enu(velocity)) if velocity else NAN3,
            acceleration=list(ned_enu(acceleration)) if acceleration else NAN3,
            jerk=NAN3, yaw=yaw, yawspeed=yawspeed))

    def follow_yaw(self, wanted, current, now):
        """ego-planner's NED yaw, turned towards at YAW_RATE_MAX_RAD_S at most from the drone's own: (yaw, rate)."""
        last, at = self.yaw if self.yaw is not None else (current, now)
        dt = (now - at) / US_PER_S
        error = (wanted - last + math.pi) % (2.0 * math.pi) - math.pi
        turn = max(-YAW_RATE_MAX_RAD_S * dt, min(YAW_RATE_MAX_RAD_S * dt, error))
        flown = (last + turn + math.pi) % (2.0 * math.pi) - math.pi
        self.yaw = (flown, now)
        return flown, (turn / dt if dt > 0.0 else 0.0)

    def publish_brake(self, yaw):
        self.mode_pub.publish(OffboardControlMode(timestamp=self.px4_now, velocity=True))
        self.setpoint_pub.publish(TrajectorySetpoint(timestamp=self.px4_now, position=NAN3, velocity=[0.0] * 3,
                                                     acceleration=NAN3, jerk=NAN3, yaw=yaw, yawspeed=math.nan))

    def on_position_cmd(self, msg):
        if self.px4_now is None:
            return
        if self.cmd is not None and speed(self.cmd.velocity) > EMERGENCY_FROM_MS and speed(msg.velocity) == 0.0:
            self.braking = True
            self.get_logger().warn(f"standstill commanded at {speed(self.cmd.velocity):.1f} m/s; braking where the drone is")
        self.cmd, self.cmd_at = msg, self.px4_now

    def command(self, command, param1, param2=0.0):
        msg = VehicleCommand(command=command, param1=param1, param2=param2,
                             target_system=self.drone_id + 1,  # rcS: MAV_SYS_ID = instance + 1
                             target_component=COMMANDER_COMPONENT, source_system=SOURCE_SYSTEM,
                             source_component=SOURCE_COMPONENT, from_external=True)
        self.pending, self.rejections = [msg, self.px4_now, 0], 0

    def resend(self):
        if self.pending is None or self.px4_now < self.pending[1]:
            return
        msg, _, unanswered = self.pending
        if unanswered == COMMAND_ATTEMPTS:
            raise RuntimeError(f"command {msg.command}: no acknowledgement from PX4 after {unanswered} sends")
        msg.timestamp = self.px4_now
        self.command_pub.publish(msg)
        self.pending = [msg, self.px4_now + COMMAND_RETRY_S * US_PER_S, unanswered + 1]

    def on_ack(self, ack):
        if self.pending is None or ack.command != self.pending[0].command:
            return
        if ack.result == VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS:
            return
        if ack.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.rejections += 1
            if self.rejections > REJECTED_RETRIES:
                raise RuntimeError(f"command {ack.command}: refused {self.rejections} times, last result {ack.result}")
            self.get_logger().warn(f"command {ack.command} refused (result {ack.result}); again in {REJECTED_RETRY_S} s")
            self.pending = [self.pending[0], self.px4_now + REJECTED_RETRY_S * US_PER_S, 0]
            return
        self.pending = None
        if ack.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.get_logger().info("OFFBOARD accepted; arming")
            self.state = "arm"
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, float(VehicleCommand.ARMING_ACTION_ARM))
        elif ack.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            position, yaw = self.hold
            self.hold = ((position[0], position[1], position[2] + self.takeoff_height), yaw)
            self.state = "takeoff"
            self.get_logger().info(f"armed; taking off {self.takeoff_height} m")


def main():
    rclpy.init()
    node = Px4Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
