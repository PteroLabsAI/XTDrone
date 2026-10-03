"""Fly the formation the way the keyboard would, unattended, and report how close every follower got to its slot.

    python3 fly_formation.py <uav_type> <uav_num>

OFFBOARD and ARM to every vehicle, the leader climbs and hovers, then each formation of formation_dict in
turn, back to "origin", the leader cruising forward and back, then AUTO.LAND. A follower's error is its distance from leader + its column of /xtdrone/formation_pattern,
both taken from PX4's global position, averaged over the end of each formation's dwell.
"""
import math
import sys
import time

import numpy
import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from formation_dict import formations_for
from geo import enu_between
from px4 import QOS, px4_topic

READY_TIMEOUT_S = 300.0  # PX4 boot and GPS settling; chosen, generous: 18 PX4 clients reach the Agent in ~11 s
MODE_RETRY_S = 2.0  # how often a refused OFFBOARD or ARM is asked again; chosen, no evidence
CLIMB_MS = 1.0  # chosen: an unhurried climb
# The deepest slot in formation_dict_6 is 6 m under the leader ("T"); chosen to keep it 4 m off the ground.
CLIMB_TO_M = 10.0
CLIMB_TIMEOUT_S = 3.0 * CLIMB_TO_M / CLIMB_MS  # chosen, generous
DWELL_S = 40.0  # the longest move in formation_dict_6 is ~8 m; chosen with a wide margin
MEASURE_S = 10.0  # the tail of each dwell that is averaged; chosen
CRUISE_MS = 3.0  # the leader's speed for the following test; chosen, above the old 1 m/s follower cap
CRUISE_S = 20.0  # chosen: a few seconds to get up to speed, then MEASURE_S of steady flight
LAND_TIMEOUT_S = 60.0  # 10 m at PX4's MPC_LAND_SPEED plus auto-disarm; chosen, generous
SPIN_S = 0.1  # chosen, no evidence


def armed(status):
    return status.arming_state == VehicleStatus.ARMING_STATE_ARMED


def timed_out(seconds, what):
    sys.exit("timed out after %gs waiting for %s; px4_<i>.log in the run's log folder says why" % (seconds, what))


class Pilot(Node):
    def __init__(self, uav_type, uav_num):
        super().__init__("fly_formation")
        self.n = uav_num
        self.status = [None] * uav_num
        self.fix = [None] * uav_num
        self.leader_position = None
        self.pattern = None
        self.cmd = []
        for i in range(uav_num):
            name = uav_type + '_' + str(i)
            self.create_subscription(VehicleStatus, px4_topic(name, "out", "vehicle_status", VehicleStatus),
                                     lambda m, i=i: self.status.__setitem__(i, m), QOS)
            self.create_subscription(VehicleGlobalPosition, px4_topic(name, "out", "vehicle_global_position", VehicleGlobalPosition),
                                     lambda m, i=i: self.fix.__setitem__(i, m), QOS)
            self.cmd.append(self.create_publisher(String, "/xtdrone/" + name + "/cmd", 3))
        self.create_subscription(VehicleLocalPosition, px4_topic(uav_type + "_0", "out", "vehicle_local_position", VehicleLocalPosition),
                                 lambda m: setattr(self, "leader_position", m), QOS)
        self.create_subscription(Float32MultiArray, "/xtdrone/formation_pattern",
                                 lambda m: setattr(self, "pattern", numpy.array(m.data, dtype=float).reshape(3, uav_num - 1)), 10)
        self.leader_vel = self.create_publisher(Twist, "/xtdrone/leader/cmd_vel_flu", 1)
        self.leader_cmd = self.create_publisher(String, "/xtdrone/leader/cmd", 1)

    def spin_until(self, done, timeout, what):
        end = time.time() + timeout
        while not done():
            if time.time() > end:
                timed_out(timeout, what)
            rclpy.spin_once(self, timeout_sec=SPIN_S)

    def spin_for(self, seconds, each=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=SPIN_S)
            if each:
                each()

    def to_all(self, text):
        for pub in self.cmd:
            pub.publish(String(data=text))

    def to_all_until(self, text, done, timeout, what):
        """Ask again until done(): PX4 refuses OFFBOARD and ARM until its estimator and checks are ready."""
        end = time.time() + timeout
        while not done():
            if time.time() > end:
                timed_out(timeout, what)
            self.to_all(text)
            retry = time.time() + MODE_RETRY_S
            while time.time() < retry and not done():
                rclpy.spin_once(self, timeout_sec=SPIN_S)

    def errors(self):
        leader = self.fix[0]
        out = []
        for i in range(1, self.n):
            e, n, u = enu_between(leader, self.fix[i])
            de, dn, du = e - self.pattern[0, i - 1], n - self.pattern[1, i - 1], u - self.pattern[2, i - 1]
            out.append(math.sqrt(de * de + dn * dn + du * du))
        return out

    def report(self, formation, dwell=DWELL_S, each=lambda: None):
        samples = []
        self.spin_for(dwell - MEASURE_S, each)
        self.spin_for(MEASURE_S, lambda: (each(), samples.append(self.errors())))
        mean = numpy.mean(samples, axis=0)
        print("%-9s  mean %.2f m  worst %.2f m  |  %s" % (formation, mean.mean(), mean.max(),
              "  ".join("%d:%.2f" % (i + 1, m) for i, m in enumerate(mean))), flush=True)


def main():
    uav_type, uav_num = sys.argv[1], int(sys.argv[2])
    formations = formations_for(uav_num)
    rclpy.init()
    p = Pilot(uav_type, uav_num)

    p.spin_until(lambda: all(p.status) and all(p.fix) and p.leader_position and p.pattern is not None,
                 READY_TIMEOUT_S, "every PX4's status and global position")
    print("all %d connected" % uav_num, flush=True)
    # OFFBOARD first: the communication nodes already stream setpoints, and PX4 then arms straight into it.
    p.to_all_until("OFFBOARD", lambda: all(s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD for s in p.status),
                   READY_TIMEOUT_S, "OFFBOARD")
    p.to_all_until("ARM", lambda: all(armed(s) for s in p.status), READY_TIMEOUT_S, "ARM")
    print("armed in OFFBOARD, climbing to %g m" % CLIMB_TO_M, flush=True)

    climb = Twist()
    climb.linear.z = CLIMB_MS
    p.spin_until(lambda: p.leader_vel.publish(climb) or -p.leader_position.z >= CLIMB_TO_M, CLIMB_TIMEOUT_S, "the climb")
    p.leader_vel.publish(Twist())
    p.report("origin")

    # Back to "origin" last: the other patterns stack vehicles above each other, and AUTO.LAND descends straight down.
    for formation in [f for f in formations if f != "origin"] + ["origin"]:
        p.leader_cmd.publish(String(data=formation))
        p.report(formation)

    # Following a moving leader: forward, back to where it was, stop.
    for forward, label in ((CRUISE_MS, "cruise +%g m/s" % CRUISE_MS), (-CRUISE_MS, "cruise -%g m/s" % CRUISE_MS), (0.0, "stopped")):
        cruise = Twist()
        cruise.linear.x = forward
        # Streamed: the communication node hovers a vehicle whose velocity stops coming.
        p.report(label, CRUISE_S, lambda: p.leader_vel.publish(cruise))

    p.to_all("AUTO.LAND")
    p.spin_until(lambda: not any(armed(s) for s in p.status), LAND_TIMEOUT_S, "landing and disarm")
    print("landed", flush=True)


if __name__ == '__main__':
    main()
