"""Fly XTDrone's fixed-wing VTOL formation: three VTOLs take off together on their missions; once all three fly as
aeroplanes the followers go OFFBOARD to fw_formation_follower, which holds each figure in turn while the leader flies
its route. Then the followers climb away and return home one at a time, the leader last, and all three land.

    python3 fly_fw_formation.py --alt 120 --route 0,2500 1500,2500 --figures 3,40 1,40 2,40

--route: the leader's points, north,east m from its home, at --alt m over home; it takes off toward the first and
circles the last until it is sent home. Each follower takes off toward its own home plus the first offset, parallel to
the leader. --figures: type,seconds in order (fw_formation_follower's types: 1 column, 2 triangle, 3 line abreast).
PX4 instance i is standard_vtol_i (run_pterosim.sh): 0 leads, 1 and 2 follow. Needs ROS sourced (the figure goes out
with `ros2 topic pub`).
"""
import argparse
import math
import queue
import struct
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

GCS_PORT_BASE = 18570      # px4-rc.mavlink:11 udp_gcs_port_local = 18570 + instance
LEADER, FOLLOWERS = 0, (1, 2)
TARGET_COMPONENT = 1       # commander drops commands to any other component
HEARTBEAT_S = 1.0          # a GCS's MAVLink heartbeat rate
PX4_STREAM_S = 1.0         # PX4's HEARTBEAT and EXTENDED_SYS_STATE period on the GCS link (mavlink_main.cpp:2311, :1437)
STALE_PERIODS = 3          # chosen: that many missed and PX4 or its link is gone
# PX4 custom mode: main mode in bits 16-23, sub mode in bits 24-31 (px4_custom_mode.h)
MAIN_AUTO, MAIN_OFFBOARD = 4, 6
SUB_MISSION, SUB_RTL = 4, 5
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
EARTH_M_PER_DEG = 111320.0  # flat earth over a few km, as the law's cov_m_2_lat_long_alt
TYPE_TOPIC = "/fw_formation/type"
FOLLOWER_NODES = 2         # `ros2 topic pub -w` waits for both fw_formation_follower nodes
PUB_KEEP_ALIVE_S = 1.0     # chosen: the publisher stays for subscribers that match after it sent
PUB_TIMEOUT_S = 15.0       # chosen: discovery of both nodes plus the keep-alive

# The leader's last circle, wide enough for the inside line-abreast follower at the law's minimum airspeed.
SLOT_ASIDE_M = 30.0        # formation type 3's slots (formation_controller.cpp:68, :74)
LAW_MIN_AIRSPEED = 12.0    # m/s, min_arispd_sp (formation_controller.hpp:74)
FW_AIRSPD_TRIM = 15.0      # m/s, the leader's cruise: PX4's default (performance_model_params.c:165), kept by airframe 22003
LOITER_MARGIN_M = 100.0    # chosen: leaves the inside follower about 1 m/s above that minimum
LOITER_RADIUS_M = SLOT_ASIDE_M * FW_AIRSPD_TRIM / (FW_AIRSPD_TRIM - LAW_MIN_AIRSPEED) + LOITER_MARGIN_M

# Take-off to fixed-wing flight: the slowest auto climb, the heading alignment, the longest front transition.
MPC_TKO_SPEED = 1.5        # m/s, the take-off climb rate (multicopter_takeoff_land_params.c:57, FlightTaskAuto.cpp:816)
MIS_YAW_TMT_S = 10.0       # rc.vtol_defaults:16
VT_TRANS_TIMEOUT_S = 15.0  # vtol_att_control_params.c:159
TRANSITION_MARGIN_S = 30.0  # chosen: arming one after another, the motors' spin-up ramp

# RTL climbs to RTL_RETURN_ALT before it turns home (rtl.cpp:477, rtl_direct_mission_land.cpp:127): follower i to i steps
# above --alt, clear of the formation.
RTL_STEP_M = 30.0          # chosen

READY_TIMEOUT_S = 120.0    # chosen: PX4 boot, EKF and GPS on a fresh spawn
MISSION_TIMEOUT_S = 10.0   # chosen: a few items over loopback
COMMAND_TIMEOUT_S = 3.0    # chosen
ARM_TIMEOUT_S = 60.0       # chosen: preflight checks settle a few seconds after the GPS fix
MODE_TIMEOUT_S = 10.0      # chosen
RTL_STAGGER_S = 60.0       # chosen: 900 m apart at a 15 m/s cruise on the way home
LANDED_TIMEOUT_S = 900.0   # chosen: home from the far end of a few-km route, then the vertical landing


class Vehicle:
    """One PX4 SITL instance on its GCS link: a thread drains the link (it streams at up to 50 Hz) into the latest
    message of each type, prints PX4's warnings, and hands mission, command and parameter replies to whoever waits."""

    def __init__(self, instance):
        self.instance = instance
        self.system = instance + 1  # rcS: MAV_SYS_ID = instance + 1
        self.link = mavutil.mavlink_connection("udpout:127.0.0.1:%d" % (GCS_PORT_BASE + instance))
        self.latest = {}  # type: (message, time received)
        self.replies = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._beat, daemon=True).start()

    def _read(self):
        while True:
            m = self.link.recv_match(blocking=True, timeout=1.0)
            if m is None or m.get_srcSystem() != self.system:
                continue
            self.latest[m.get_type()] = (m, time.time())
            if m.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK", "COMMAND_ACK", "PARAM_VALUE"):
                self.replies.put(m)
            if m.get_type() == "STATUSTEXT" and m.severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
                print("standard_vtol_%d: PX4: %s" % (self.instance, m.text), flush=True)

    def _beat(self):
        while True:  # PX4 learns where to send from this
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(HEARTBEAT_S)

    def fresh(self, kind):
        """The newest message of a type, None before the first; fails once it stops coming."""
        if kind not in self.latest:
            return None
        m, at = self.latest[kind]
        if time.time() - at > STALE_PERIODS * PX4_STREAM_S:
            fail("standard_vtol_%d: no %s for %.0f s: PX4 or its link is gone" % (self.instance, kind, time.time() - at))
        return m

    def reply(self, types, timeout):
        end = time.time() + timeout
        while time.time() < end:
            try:
                m = self.replies.get(timeout=max(0.0, end - time.time()))
            except queue.Empty:
                break
            if m.get_type() in types:
                return m
        return None

    def command(self, cmd, *params):
        self.link.mav.command_long_send(self.system, TARGET_COMPONENT, cmd, 0, *(list(params) + [0.0] * (7 - len(params))))
        end = time.time() + COMMAND_TIMEOUT_S
        while time.time() < end:
            ack = self.reply({"COMMAND_ACK"}, end - time.time())
            if ack is not None and ack.command == cmd:
                return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return False

    def set_param(self, name, value):
        stored = struct.unpack("f", struct.pack("f", value))[0]  # PARAM_VALUE carries the float32 PX4 keeps
        self.link.mav.param_set_send(self.system, TARGET_COMPONENT, name.encode(), value,
                                     mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        end = time.time() + COMMAND_TIMEOUT_S
        while time.time() < end:
            m = self.reply({"PARAM_VALUE"}, end - time.time())
            if m is not None and m.param_id == name and m.param_value == stored:
                return
        fail("standard_vtol_%d did not confirm %s = %g" % (self.instance, name, value))

    def set_mode(self, main, sub=0):
        return self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def main_mode(self):
        hb = self.fresh("HEARTBEAT")
        return (hb.custom_mode >> 16) & 0xFF if hb else None

    def armed(self):
        hb = self.fresh("HEARTBEAT")
        return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def fixed_wing(self):
        s = self.fresh("EXTENDED_SYS_STATE")
        return bool(s and s.vtol_state == mavutil.mavlink.MAV_VTOL_STATE_FW)

    def landed(self):
        s = self.fresh("EXTENDED_SYS_STATE")
        return bool(s and s.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND) and not self.armed()

    def home(self):
        p = self.fresh("GLOBAL_POSITION_INT")
        return (p.lat / 1e7, p.lon / 1e7) if p and (p.lat or p.lon) else None  # 0, 0 until the GPS fix

    def upload(self, items):
        """items: (command, lat, lon, alt over home, param3) in order; the mission protocol, item by item as PX4 asks."""
        mission = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        self.link.mav.mission_count_send(self.system, TARGET_COMPONENT, len(items), mission)
        while True:
            m = self.reply({"MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"}, MISSION_TIMEOUT_S)
            if m is None:
                fail("standard_vtol_%d: no reply while uploading its mission" % self.instance)
            if m.get_type() == "MISSION_ACK":
                if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    fail("standard_vtol_%d refused its mission: MAV_MISSION_RESULT %d" % (self.instance, m.type))
                return
            cmd, lat, lon, alt, param3 = items[m.seq]
            self.link.mav.mission_item_int_send(
                self.system, TARGET_COMPONENT, m.seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd,
                1 if m.seq == 0 else 0, 1, 0, 0, param3, math.nan, int(round(lat * 1e7)), int(round(lon * 1e7)), alt,
                mission)


def fail(why):
    print("FAILED: " + why, flush=True)
    sys.exit(1)


def offset(home, north, east):
    lat, lon = home
    return lat + north / EARTH_M_PER_DEG, lon + east / (EARTH_M_PER_DEG * math.cos(math.radians(lat)))


def mission(home, points, alt):
    """VTOL take-off toward the first point, the rest, circling the last until told otherwise; a VTOL mission must end
    in a landing (rc.vtol_defaults: MIS_TKO_LAND_REQ 2), so one follows that the circle never reaches."""
    nav = mavutil.mavlink
    # PX4 transitions along the bearing to the take-off point (mission.cpp:370): random with the point underneath.
    items = [(nav.MAV_CMD_NAV_VTOL_TAKEOFF, *points[0], alt, 0.0)]
    items += [(nav.MAV_CMD_NAV_WAYPOINT, *p, alt, 0.0) for p in points[1:-1]]
    items += [(nav.MAV_CMD_NAV_LOITER_UNLIM, *points[-1], alt, LOITER_RADIUS_M),
              (nav.MAV_CMD_NAV_WAYPOINT, *home, alt, 0.0),
              (nav.MAV_CMD_NAV_VTOL_LAND, *home, 0.0, 0.0)]
    return items


def wait_for(what, done, timeout, hint=""):
    end = time.time() + timeout
    while not done():
        if time.time() > end:
            fail("%s within %.0f s%s" % (what, timeout, hint))
        time.sleep(0.2)


def publish_figure(figure):
    try:
        subprocess.run(["ros2", "topic", "pub", "--once", "-w", str(FOLLOWER_NODES), "--keep-alive", str(PUB_KEEP_ALIVE_S),
                        "--qos-reliability", "reliable", "--qos-durability", "transient_local",
                        TYPE_TOPIC, "std_msgs/msg/UInt8", "{data: %d}" % figure],
                       check=True, stdout=subprocess.DEVNULL, timeout=PUB_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        fail("figure %d: no %d fw_formation_follower nodes on %s within %.0f s" % (figure, FOLLOWER_NODES, TYPE_TOPIC,
                                                                                  PUB_TIMEOUT_S))


def pair(text, kind):
    a, b = text.split(",")
    return kind(a), float(b)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--alt", type=float, required=True, help="m over each vehicle's home")
    p.add_argument("--route", nargs="+", required=True, type=lambda t: pair(t, float), help="north,east m from the leader's home")
    p.add_argument("--figures", nargs="+", required=True, type=lambda t: pair(t, int), help="type,seconds")
    a = p.parse_args()
    transition_timeout = a.alt / MPC_TKO_SPEED + MIS_YAW_TMT_S + VT_TRANS_TIMEOUT_S + TRANSITION_MARGIN_S

    fleet = [Vehicle(i) for i in (LEADER, *FOLLOWERS)]
    wait_for("a position from every vehicle", lambda: all(v.home() for v in fleet), READY_TIMEOUT_S,
             " (QGroundControl on a GCS port takes the link: PX4 answers its first partner only, mavlink_receiver.cpp:3200)")
    homes = [v.home() for v in fleet]
    for v, home in zip(fleet, homes):
        if v.instance == LEADER:
            points = [offset(home, n, e) for n, e in a.route]
        else:
            # Each follower heads out parallel to the leader's first leg from its own home, to circle at its end if it
            # never goes OFFBOARD; it shares no point with the leader's route.
            points = [offset(home, *a.route[0])]
        v.upload(mission(home, points, a.alt))
        print("standard_vtol_%d: mission uploaded, home %.6f %.6f" % (v.instance, *home), flush=True)

    for v in fleet:
        if not v.set_mode(MAIN_AUTO, SUB_MISSION):
            fail("standard_vtol_%d refused AUTO.MISSION" % v.instance)
    end = time.time() + ARM_TIMEOUT_S
    for v in fleet:
        while not v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            if time.time() > end:
                fail("standard_vtol_%d did not arm" % v.instance)
            time.sleep(1.0)
    print("armed; taking off", flush=True)

    wait_for("all three in fixed-wing flight", lambda: all(v.fixed_wing() for v in fleet), transition_timeout)
    publish_figure(a.figures[0][0])
    for i in FOLLOWERS:
        v = fleet[i]
        end = time.time() + MODE_TIMEOUT_S
        while not (v.set_mode(MAIN_OFFBOARD) and v.main_mode() == MAIN_OFFBOARD):
            if time.time() > end:
                fail("standard_vtol_%d did not take OFFBOARD (is fw_formation_follower streaming?)" % i)
            time.sleep(0.5)
    print("followers in OFFBOARD", flush=True)

    for figure, seconds in a.figures:
        publish_figure(figure)
        print("figure %d for %.0f s" % (figure, seconds), flush=True)
        end = time.time() + seconds
        while time.time() < end:
            for i in FOLLOWERS:
                if fleet[i].main_mode() != MAIN_OFFBOARD:
                    fail("standard_vtol_%d left OFFBOARD (custom main mode %s)" % (i, fleet[i].main_mode()))
            time.sleep(0.5)

    for i in (*FOLLOWERS, LEADER):
        if i != LEADER:
            fleet[i].set_param("RTL_RETURN_ALT", a.alt + i * RTL_STEP_M)
        if not fleet[i].set_mode(MAIN_AUTO, SUB_RTL):
            fail("standard_vtol_%d refused RTL" % i)
        print("standard_vtol_%d returning home" % i, flush=True)
        if i != LEADER:
            time.sleep(RTL_STAGGER_S)
    wait_for("all three landed and disarmed", lambda: all(v.landed() for v in fleet), LANDED_TIMEOUT_S)
    print("all landed", flush=True)


if __name__ == "__main__":
    main()
