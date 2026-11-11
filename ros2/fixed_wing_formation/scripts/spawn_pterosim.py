"""Spawn a three-vehicle PteroSim formation.

    python spawn_pterosim.py <east_m> <north_m> <ground_m> <heading_deg>

Always spawns a quadtailsitter, tiltrotor and explora formation. Coordinates are metres;
existing aircraft are removed.
"""
import math
import sys

from pterosim import PteroSim

CM_PER_M = 100.0
# Keep slot order aligned with the leader and the two follower nodes.
AIRCRAFT_MODELS = (("quadtailsitter", 90.0), ("tiltrotor", 0.0), ("explora", 90.0))
BEHIND_M, ASIDE_M = 15.0, 15.0
SLOTS = (
    (0.0, 0.0),
    (-BEHIND_M, ASIDE_M),
    (-BEHIND_M, -ASIDE_M),
)  # (ahead, right); follower 1 is to the leader's right.
SPAWN_CLEAR_M = 0.5
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
        for aircraft in sim.aircraft_status():
            sim.get_aircraft(aircraft.instance_id).remove()
        for (aircraft_name, pitch), (ahead, right) in zip(AIRCRAFT_MODELS, SLOTS):
            e = east + ahead * math.sin(h) + right * math.cos(h)
            n = north + ahead * math.cos(h) - right * math.sin(h)
            # Unreal: Y is south; yaw 0 faces east and 90 faces south.
            vehicle = sim.spawn(
                aircraft_name,
                x=e * CM_PER_M,
                y=-n * CM_PER_M,
                z=(ground + SPAWN_CLEAR_M) * CM_PER_M,
                yaw=heading - 90.0,
                pitch=pitch,
            )
            stack = vehicle.set_flight_stack("PX4")
            for sensor in vehicle.list_sensors():
                if sensor.type == "camera":
                    vehicle.remove_sensor(sensor.name)
            print(f"{aircraft_name} ({stack}) instance {vehicle.instance_id} at E{e:+.1f} N{n:+.1f}")
        sim.set_time_scale(TIME_SCALE)
        sim.start()
    finally:
        sim.close()


if __name__ == "__main__":
    main()
