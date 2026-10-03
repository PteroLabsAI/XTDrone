"""Put the formation's fleet into a running PteroSim: the leader and its followers on the "origin" pattern.

    python spawn_pterosim.py <aircraft> <uav_num> <east_m> <north_m>

The leader goes to <east_m>, <north_m> in the map's metres (Unreal X east, Y south); pick a flat clear spot,
since a spawn sits on whatever is under it and the fleet later lands back on the same grid.

Runs where PteroSim runs (its scripting server listens on 127.0.0.1:10010). Every aircraft already in
the scene is removed first. Real time: these nodes tick on the wall clock, so a faster simulation would
stretch their control loops in simulated time.
"""
import sys

from pterosim import PteroSim

from formation_dict import formations_for

CM_PER_M = 100.0  # spawn() takes Unreal units
TIME_SCALE = 1.0


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    aircraft, uav_num, east, north = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    # The leader sits at the pattern's zero; formation_dict is ENU metres.
    grid = [(east, north)] + [(east + float(e), north + float(n)) for e, n, _ in formations_for(uav_num)["origin"].T]

    sim = PteroSim("127.0.0.1:10010")
    try:
        if sim.status().is_running:
            sim.stop()
        for a in sim.aircraft_status():
            sim.get_aircraft(a.instance_id).remove()
        for e, n in grid:
            # Unreal X is east and Y is south.
            drone = sim.spawn(aircraft, x=e * CM_PER_M, y=-n * CM_PER_M, z=0.0, yaw=0.0)
            # The formation needs no pictures, and every camera is another render per frame.
            for s in drone.list_sensors():
                if s.type == "camera":
                    drone.remove_sensor(s.name)
            print(f"{aircraft} instance {drone.instance_id} at E{e:+.1f} N{n:+.1f}")
        sim.set_time_scale(TIME_SCALE)
        sim.start()
    finally:
        sim.close()


if __name__ == '__main__':
    main()
