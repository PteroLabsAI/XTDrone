"""A flight for the camera: the fleet lifts off together, flies forward along the runway and changes formation on the
move, then stops and lands. Prints the followers' mean slot error every second, to cut on the settled figures.

    python3 fly_show.py <uav_type> <uav_num>      (from fomation_demo/, next to fly_formation.py)
"""
import sys
import time

import numpy
import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import String

from fly_formation import Pilot, armed, READY_TIMEOUT_S

CLIMB_MS = 2.0          # chosen: a brisk lift-off; followers get the leader's velocity whole
CLIMB_TO_M = 12.0       # the sphere's lowest slot is 6 m under the leader (formation_dict_18): 6 m clear
CLIMB_TIMEOUT_S = 3.0 * CLIMB_TO_M / CLIMB_MS   # chosen, generous
HOLD_S = 3.0            # chosen: the grid hangs still before it moves off
CRUISE_MS = 7.0         # chosen: brisk forward flight; the whole show stays on the runway's ~530 m from its west end
# A follower takes the leader's measured velocity, so it lags ~tau * dv; a step to 7 m/s left the ones ahead 4 m behind
# and the leader flew through one (take swarm_show1: 0.27 m). Ramped, the 1 m/s correction keeps up.
RAMP_S = 7.0            # chosen: 1 m/s^2
# Seconds per figure: settling at the followers' 2 m/s measured in take swarm_show5 (cuboid 8.6, sphere 6.6, diamond
# 5.1, origin 8.2) plus HOLD_FORMED_S with the figure formed: v3 broke each figure up as soon as it formed and the
# user found the changes over too quickly.
HOLD_FORMED_S = 4.5             # chosen: time to show a formed figure from a second camera
FIGURES = [(name, settle + HOLD_FORMED_S) for name, settle in
           (("cuboid", 8.6), ("sphere", 6.6), ("diamond", 5.1), ("origin", 8.2))]   # origin last: AUTO.LAND goes straight down
STOP_S = 6.0            # chosen: the fleet settles after the ramp down, before landing
LAND_TIMEOUT_S = 60.0   # as fly_formation
REPORT_S = 1.0


def main():
    uav_type, uav_num = sys.argv[1], int(sys.argv[2])
    rclpy.init()
    p = Pilot(uav_type, uav_num)
    t0 = [None]
    last = [0.0]

    def report(label):
        now = time.time()
        if now - last[0] >= REPORT_S and p.pattern is not None and all(p.fix):
            last[0] = now
            e = p.errors()
            lp = p.leader_position
            print("%6.1f s  %-8s err mean %.2f max %.2f m  leader N %.1f E %.1f up %.1f  v %.1f m/s" % (
                now - t0[0], label, numpy.mean(e), max(e), lp.x, lp.y, -lp.z, (lp.vx ** 2 + lp.vy ** 2) ** 0.5), flush=True)

    def stream(twist, seconds, label):
        end = time.time() + seconds
        while time.time() < end:
            p.leader_vel.publish(twist)
            rclpy.spin_once(p, timeout_sec=0.05)
            report(label)

    def ramp(v0, v1, label):
        start = time.time()
        while time.time() - start < RAMP_S:
            twist = Twist()
            twist.linear.x = v0 + (v1 - v0) * (time.time() - start) / RAMP_S
            p.leader_vel.publish(twist)
            rclpy.spin_once(p, timeout_sec=0.05)
            report(label)

    p.spin_until(lambda: all(p.status) and all(p.fix) and p.leader_position and p.pattern is not None,
                 READY_TIMEOUT_S, "every PX4's status and global position")
    print("all %d connected" % uav_num, flush=True)
    p.to_all_until("OFFBOARD", lambda: all(s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD for s in p.status),
                   READY_TIMEOUT_S, "OFFBOARD")
    p.to_all_until("ARM", lambda: all(armed(s) for s in p.status), READY_TIMEOUT_S, "ARM")
    t0[0] = time.time()
    print("armed in OFFBOARD", flush=True)

    climb = Twist()
    climb.linear.z = CLIMB_MS
    p.spin_until(lambda: p.leader_vel.publish(climb) or report("climb") or -p.leader_position.z >= CLIMB_TO_M,
                 CLIMB_TIMEOUT_S, "the climb")
    print("%6.1f s  MARK climbed" % (time.time() - t0[0]), flush=True)
    stream(Twist(), HOLD_S, "hold")

    cruise = Twist()
    cruise.linear.x = CRUISE_MS
    print("%6.1f s  MARK ramp up" % (time.time() - t0[0]), flush=True)
    ramp(0.0, CRUISE_MS, "ramp up")
    for figure, seconds in FIGURES:
        p.leader_cmd.publish(String(data=figure))
        print("%6.1f s  MARK %s" % (time.time() - t0[0], figure), flush=True)
        stream(cruise, seconds, figure)

    print("%6.1f s  MARK ramp down" % (time.time() - t0[0]), flush=True)
    ramp(CRUISE_MS, 0.0, "ramp down")
    stream(Twist(), STOP_S, "stop")
    p.to_all("AUTO.LAND")
    print("%6.1f s  MARK land" % (time.time() - t0[0]), flush=True)
    p.spin_until(lambda: report("land") or not any(armed(s) for s in p.status), LAND_TIMEOUT_S, "landing and disarm")
    print("%6.1f s  landed" % (time.time() - t0[0]), flush=True)


if __name__ == '__main__':
    main()
