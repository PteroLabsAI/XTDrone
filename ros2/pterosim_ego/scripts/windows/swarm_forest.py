"""EGO-Planner swarm of N PteroSim x500s line abreast through a forest of the map playing in PteroSim: spawns them with
depth cameras on a line across the heading, gives each the point --length ahead of its spawn as its goal, starts the
WSL side (../wsl/run_swarm.sh) and waits until every drone is at its goal, printing the real-time factor. The planners
find their way around the trees from their own depth cameras.

    python swarm_forest.py --start x,y --heading deg --length m --drones N --fly-height m
        --distro <WSL distro> --px4 <px4_root> --px4-ros-ws <dir> --ego-ws <dir>

--start is the line's middle in metres UE, --heading the flight's UE yaw; --px4, --px4-ros-ws and --ego-ws are paths in
WSL. Each drone takes off --fly-height above its spawn and flies to that height above its own spawn's ground; the
planner's floor is the lowest spawn. GPS noise is off on every drone: a swarm flies on RTK, and independent 1.7 m
errors put neighbours 5 m apart into each other.
"""
import argparse
import math
import os
import subprocess
import sys
import time

from pterosim import PteroSim

CM_PER_M = 100.0  # spawn() and aircraft_status() are in Unreal units
CONTROL = "127.0.0.1:10010"  # PteroSim's scripting server; mirrored WSL reaches it on loopback for pterosim_clock
SPACING_M = 5.0  # chosen: several times the launch's 1.12 m swarm clearance, so neighbours swerve around trunks apart
MAP_PAD_M = 10.0  # chosen: room around the spawns and goals in the planner's map
CEILING_PAD_M = 2.5  # chosen: above the highest goal; lower keeps the planner from climbing over the trees
DEPTH_PORT = 5710  # chosen; drone i's camera serves it + 2 * i (pterosim_ego depth_bridge)
CAMERA_FRD_M = (0.15, 0.0, 0.0)  # the launch's camera_offset_frd
MAX_RANGE_M = 10.0  # chosen: past the launch's 8 m ray length
DEPTH_W, DEPTH_H = 320, 240  # chosen: 640x480 16-bit depth at 30 Hz is 18 MB/s per drone in 1400-byte datagrams
DEPTH_HZ = 30  # the sensor's default
GOAL_REACHED_M = 3.0  # chosen
MIN_MEAN_SPEED_MS = 1.0  # chosen: a fifth of the launch's 5 m/s limit; slower than this on average, the swarm is stuck
TIME_SCALE = 1.0  # chosen: real time; PX4 and the planners follow the simulation's clock at whatever rate it reaches
PRINT_S = 10.0  # chosen
POLL_S = 1.0  # chosen


def fmt(points):
    return ";".join(",".join(f"{v:g}" for v in p) for p in points)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for name, kind in (("--start", str), ("--heading", float), ("--length", float), ("--drones", int),
                       ("--fly-height", float), ("--distro", str), ("--px4", str), ("--px4-ros-ws", str),
                       ("--ego-ws", str)):
        ap.add_argument(name, type=kind, required=True)
    a = ap.parse_args()
    sx, sy = (float(v) for v in a.start.split(","))
    th = math.radians(a.heading)
    ux, uy = math.cos(th), math.sin(th)
    offsets = [(i - (a.drones - 1) / 2) * SPACING_M for i in range(a.drones)]
    spawns_ue = [(sx - o * uy, sy + o * ux) for o in offsets]
    goals_ue = [(x + a.length * ux, y + a.length * uy) for x, y in spawns_ue]
    ox, oy = sx + a.length / 2 * ux, sy + a.length / 2 * uy  # the swarm world's origin: the middle of the planners' maps
    enu = lambda x, y: (x - ox, -(y - oy))  # noqa: E731  UE +y is south

    sim = PteroSim(CONTROL)
    try:
        sim.stop()
        for s in sim.aircraft_status():
            sim.get_aircraft(s.instance_id).remove()
        drones = []
        for i, (x, y) in enumerate(spawns_ue):
            d = sim.spawn("x500", x=x * CM_PER_M, y=y * CM_PER_M, z=0.0, yaw=a.heading)
            if d.instance_id != i:
                sys.exit(f"drone {i} spawned as instance {d.instance_id}: the launch needs instances 0..N-1")
            for s in d.list_sensors():
                if s.type == "camera":
                    d.remove_sensor(s.name)  # every camera is another render
                elif s.type == "gps":
                    d.set_sensor_param(s.name, noise=False)
            depth = d.add_sensor("depthcamera")
            d.set_sensor_pose(depth, CAMERA_FRD_M)
            d.set_sensor_param(depth, update_hz=DEPTH_HZ, image_width=DEPTH_W, image_height=DEPTH_H, raw_stream=True,
                               raw_stream_port=DEPTH_PORT, max_range_m=MAX_RANGE_M, noise=False)
            drones.append(d)
        # A spawn lands on whatever surface is under it, so each drone's own height is its ground.
        status = {s.instance_id: s for s in sim.aircraft_status()}
        missing = [d.instance_id for d in drones if d.instance_id not in status]
        if missing:
            sys.exit(f"PteroSim reports no position for drones {missing} after spawning them")
        ground = [status[d.instance_id].z / CM_PER_M for d in drones]
        floor = min(ground)
        spawns_enu = [(*enu(x, y), g - floor) for (x, y), g in zip(spawns_ue, ground)]
        goals_enu = [(*enu(x, y), g - floor + a.fly_height) for (x, y), g in zip(goals_ue, ground)]
        ceiling = max(z for _, _, z in goals_enu) + CEILING_PAD_M
        reach_x = max(abs(p[0]) for p in spawns_enu + goals_enu) + MAP_PAD_M
        reach_y = max(abs(p[1]) for p in spawns_enu + goals_enu) + MAP_PAD_M
        map_size = (2 * reach_x, 2 * reach_y, ceiling + CEILING_PAD_M)
        print(f"map {map_size[0]:.0f} x {map_size[1]:.0f} x {map_size[2]:.0f} m, ceiling {ceiling:.1f} m", flush=True)
        sim.set_time_scale(TIME_SCALE)
        sim.start()

        wsl = subprocess.run(["wsl.exe", "-d", a.distro, "-e", "wslpath", "-a",
                              os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "wsl")).replace("\\", "/")],
                             capture_output=True, text=True, check=True).stdout.strip()
        subprocess.run(["wsl.exe", "-d", a.distro, "-e", "bash", f"{wsl}/run_swarm.sh", a.px4, a.px4_ros_ws, a.ego_ws,
                        str(a.drones), f"raw_stream_port:={DEPTH_PORT}", f"control:={CONTROL}",
                        f"spawns:={fmt(spawns_enu)}", f"goals:={fmt(goals_enu)}",
                        f"map_size:={map_size[0]:.0f},{map_size[1]:.0f},{map_size[2]:.0f}", f"ceiling:={ceiling:.1f}",
                        f"depth_width:={DEPTH_W}", f"depth_height:={DEPTH_H}", f"takeoff_height:={a.fly_height:g}"],
                       check=True)

        t0, sim_t0 = time.time(), sim.clock().simulation_time
        last_wall, last_sim = t0, sim_t0
        while True:
            now = {s.instance_id: s for s in sim.aircraft_status()}
            off = [math.hypot(now[d.instance_id].x / CM_PER_M - gx, now[d.instance_id].y / CM_PER_M - gy)
                   for d, (gx, gy) in zip(drones, goals_ue)]
            sim_t = sim.clock().simulation_time
            if all(o < GOAL_REACHED_M for o in off):
                print(f"all {a.drones} at their goals after {sim_t - sim_t0:.0f} s of simulation", flush=True)
                return
            if sim_t - sim_t0 > a.length / MIN_MEAN_SPEED_MS:
                sys.exit(f"timeout; off the goals {[round(o) for o in off]} m")
            if time.time() - last_wall >= PRINT_S:
                h = [now[d.instance_id].z / CM_PER_M - g for d, g in zip(drones, ground)]
                print(f"{sim_t - sim_t0:5.0f} s  RTF {(sim_t - last_sim) / (time.time() - last_wall):4.2f}  to goal "
                      + " ".join(f"{o:4.0f}" for o in off) + "  h " + " ".join(f"{v:4.1f}" for v in h), flush=True)
                last_wall, last_sim = time.time(), sim_t
            time.sleep(POLL_S)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
