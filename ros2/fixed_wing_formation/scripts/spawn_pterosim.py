"""Put the fixed-wing formation's three VTOLs into a running PteroSim, as the original's launch file placed them:
the leader at a point, the followers 15 m behind it and 15 m to either side, all facing the first leg.

    python spawn_pterosim.py <east_m> <north_m> <ground_m> <heading_deg>

<east_m>, <north_m>: the leader's spot in the map's metres (Unreal X east, Y south); <ground_m>: the ground's height
there in the map (Unreal Z / 100), the same under all three, so pick a flat clear spot; <heading_deg>: compass heading
of the route's first leg. Every aircraft already in the scene is removed first. Real time: the follower node's TECS
steps on the wall clock.
"""
import math
import sys

from pterosim import PteroSim

AIRCRAFT = "standard_vtol"  # PX4 airframe 22003
CM_PER_M = 100.0
# fixed_wing_formation_control.launch: uav0 at (-1500, 0), uav1 at (-1515, -15), uav2 at (-1515, 15), facing east
BEHIND_M, ASIDE_M = 15.0, 15.0
SLOTS = ((0.0, 0.0), (-BEHIND_M, ASIDE_M), (-BEHIND_M, -ASIDE_M))   # (ahead, right) of the leader: uav1 on the right
SPAWN_CLEAR_M = 0.5        # chosen: the gear settles onto the ground from here
TIME_SCALE = 1.0


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    east, north, ground, heading = (float(v) for v in sys.argv[1:])
    h = math.radians(heading)
    sim = PteroSim("127.0.0.1:10010")
    try:
        if sim.status().is_running:
            sim.stop()
        for a in sim.aircraft_status():
            sim.get_aircraft(a.instance_id).remove()
        for ahead, right in SLOTS:
            e = east + ahead * math.sin(h) + right * math.cos(h)
            n = north + ahead * math.cos(h) - right * math.sin(h)
            # Unreal: Y is south, yaw 0 faces east and 90 south, so a compass heading is yaw + 90.
            v = sim.spawn(AIRCRAFT, x=e * CM_PER_M, y=-n * CM_PER_M, z=(ground + SPAWN_CLEAR_M) * CM_PER_M, yaw=heading - 90.0)
            for s in v.list_sensors():  # the formation needs no pictures, and each camera is another render a frame
                if s.type == "camera":
                    v.remove_sensor(s.name)
            print(f"{AIRCRAFT} instance {v.instance_id} at E{e:+.1f} N{n:+.1f}")
        sim.set_time_scale(TIME_SCALE)
        sim.start()
    finally:
        sim.close()


if __name__ == "__main__":
    main()
