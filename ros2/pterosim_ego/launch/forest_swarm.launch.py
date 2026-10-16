"""N PteroSim x500s through a forest: PX4 SITL over uXRCE-DDS, PteroSim depth cameras, ego-planner-swarm.

    ros2 launch pterosim_ego forest_swarm.launch.py raw_stream_port:=5710 control:=127.0.0.1:10010 \
        spawns:="-45,-3,0;-45,3,0" goals:="45,3,2;45,-3,2"

scripts/windows/swarm_forest.py computes these for a line of drones and starts it through scripts/wsl/run_swarm.sh. Drone i is
PteroSim instance i, PX4 instance i (namespace <px4_ns_prefix><i>) and ego-planner drone i, ids 0..N-1: each planner
starts after the one before it (ego_replan_fsm.cpp:77). spawns and goals are x,y,z metres in the swarm's ENU world,
whose origin is the middle of the map (grid_map.cpp:92). The clock or an agent exiting stops the launch; a drone's own
nodes ending (its crash closes its camera's stream) leave the rest flying.
"""
import math
import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessIO

from pterosim_ego import drone_topic

XRCE_AGENT_PORT = 8888  # PX4 rcS's uxrce_dds_port; agent g serves port XRCE_AGENT_PORT + g
# Chosen: an agent serves all its clients on one thread; px4_swarm.sh gives instance i port + i // this.
DRONES_PER_AGENT = 2
INCH_M = 0.0254
# x500 prop tips 0.41 m from the centre: rotors at (+-0.174, +-0.174) m (PteroSimAircrafts x500/Propulsion.xml), 13 in
# props (x500/Engines/x500_prop_0.xml).
X500_TIP_RADIUS_M = math.hypot(0.174, 0.174) + 13.0 * INCH_M / 2
# Chosen: room for PX4's sideways error on ego-planner's commands at speed; a wider margin closes the gaps between
# this forest's trunks and puts waypoints inside crowns, a narrower one lets the props reach branches.
TRACKING_MARGIN_M = 0.3
# ego-planner treats the drone as a point (upstream inflation 0.099 m), so obstacles grow by the x500's reach.
OBSTACLES_INFLATION_M = X500_TIP_RADIUS_M + TRACKING_MARGIN_M
# The optimizer rejects a plan passing another drone's centre closer than this, and pushes for twice it
# (bspline_optimizer.cpp:878, :1596): two x500s' prop tips never meet.
SWARM_CLEARANCE_M = 2 * X500_TIP_RADIUS_M + TRACKING_MARGIN_M
# ego-planner-swarm's own values (plan_manage/launch/advanced_param.launch.py:115-200, run_in_sim.launch.py:59-61, :98)
# but for the constants below.
MAX_VEL_MS = 5.0  # chosen: PX4 lags the plan's turns between trunks more the faster it flies; this bounds the lag
MAX_ACC_MS2 = 4.0  # chosen: stops from 5 m/s in 3.1 m, inside the 8 m map ray
MAX_JERK_MS3 = 8.0  # chosen
# Upstream's 0.1 lets plans exceed MAX_ACC sideways; chosen tenfold to weight the feasibility cost toward the limits.
LAMBDA_FEASIBILITY = 1.0
PLANNING_HORIZON_M = 10.0  # chosen: past the 8 m ray
RAY_LENGTH_M = 8.0  # chosen: room to stop from MAX_VEL_MS (3.1 m) plus the replan; the camera reaches 10 m
LOCAL_UPDATE_M = 9.0  # chosen: the map around the drone that the camera refreshes, past the ray length
# 0.15, not 0.1: eight planners on a 116 x 96 m map would hold 2.8 GB each at 0.1.
GRID_RESOLUTION_M = 0.15
# Chosen: ego latches EMERGENCY_STOP for good on a longer gap (grid_map.cpp:716-724), and a swarm's start, every PX4
# and planner coming up at once, can hold odometry back for more than upstream's 1 s.
ODOM_DEPTH_TIMEOUT_S = 3.0
# Chosen: ego takes the next waypoint once this near the one it flies to; at upstream's 1 m it braked at each (its global
# trajectory ends every waypoint at rest).
NEXT_WAYPOINT_M = 3.0
TRAJ_TIME_FORWARD_S = 1.0
PRESET_TARGET = 2  # fsm/flight_type: the goal is waypoint 0 (ego_replan_fsm.h TARGET_TYPE)
POSE_STAMPED = 1  # grid_map/pose_type: the camera's pose, paired with each depth image (grid_map.h:155)


def _points(text, name):
    points = [tuple(float(v) for v in p.split(",")) for p in text.split(";")]
    if any(len(p) != 3 for p in points):
        raise ValueError(f"{name}:={text!r}: each point is x,y,z in metres, points separated by ';'")
    return points


def _routes(text):
    """Each drone's waypoints: x,y,z m, '/' between waypoints, ';' between drones."""
    if "/" not in text:
        return [[p] for p in _points(text, "goals")]
    return [_points(route.replace("/", ";"), "goals") for route in text.split(";") if route]


def _check_in_map(points, name, map_size, ceiling):
    half_x, half_y = map_size[0] / 2, map_size[1] / 2
    for x, y, z in points:
        if not (-half_x < x < half_x and -half_y < y < half_y and z < ceiling):
            raise ValueError(f"{name}: ({x}, {y}, {z}) is outside the map +-{half_x} x +-{half_y} m below the "
                             f"{ceiling} m ceiling")


def _agent(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(("0.0.0.0", port))
        except OSError:
            return LogInfo(msg=f"UDP {port} is taken: using the Micro XRCE-DDS agent already running")
    return ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", str(port)], name=f"xrce_agent_{port}", output="log",
                          on_exit=Shutdown(reason=f"xrce_agent on {port} exited"))


def _node(name, **kwargs):
    # A drone's node ends with its drone (a crash closes its camera's stream): the rest of the swarm flies on.
    return Node(name=name, on_exit=LogInfo(msg=f"{name} exited"), **kwargs)


def _drones(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    map_size = tuple(float(v) for v in arg("map_size").split(","))
    ceiling = float(arg("ceiling"))
    takeoff_height = float(arg("takeoff_height"))
    spawns, routes = _points(arg("spawns"), "spawns"), _routes(arg("goals"))
    goals = [w for route in routes for w in route]
    if len(spawns) != len(routes):
        raise ValueError(f"{len(spawns)} spawns for {len(routes)} routes")
    _check_in_map(spawns, "spawns", map_size, ceiling)
    _check_in_map(goals, "goals", map_size, ceiling)
    if not all(OBSTACLES_INFLATION_M < z for _, _, z in goals) or takeoff_height >= ceiling:
        raise ValueError(f"goals' z and takeoff_height must sit between the ground's {OBSTACLES_INFLATION_M:.2f} m "
                         f"inflation and the {ceiling} m ceiling")
    width, height, fov_deg = int(arg("depth_width")), int(arg("depth_height")), float(arg("depth_fov_deg"))
    # The depth camera's pinhole, as the sensor stamps every frame (PteroSimSensors BaseRenderSensor.cpp:339-341).
    fx = fy = width / 2 / math.tan(math.radians(fov_deg) / 2)
    cx, cy = (width - 1) / 2, (height - 1) / 2
    camera_offset_frd = [float(v) for v in arg("camera_offset_frd").split(",")]
    sim_time = {"use_sim_time": True}

    actions = [_agent(XRCE_AGENT_PORT + g) for g in range(math.ceil(len(spawns) / DRONES_PER_AGENT))]
    # The simulation's own clock for every node on use_sim_time; no clock, no planning: its exit ends the swarm.
    actions.append(Node(package="pterosim_ego", executable="pterosim_clock", name="pterosim_clock", output="screen",
                        parameters=[{"control": arg("control")}],
                        on_exit=Shutdown(reason="pterosim_clock exited")))
    for i, (spawn, route) in enumerate(zip(spawns, routes)):
        odom = drone_topic(i, "visual_slam/odom")
        depth = drone_topic(i, "pcl_render_node/depth")
        camera_pose = drone_topic(i, "pcl_render_node/camera_pose")
        bspline = drone_topic(i, "planning/bspline")
        position_cmd = drone_topic(i, "planning/pos_cmd")
        bridges = [
            _node(package="pterosim_ego", executable="depth_bridge", name=f"drone_{i}_depth_bridge",
                 parameters=[sim_time, {"drone_id": i, "host": arg("raw_stream_host"),
                                        "raw_stream_port": int(arg("raw_stream_port")),
                                        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
                                        "camera_offset_frd": camera_offset_frd}],
                 remappings=[("depth", depth)]),
            _node(package="pterosim_ego", executable="px4_bridge", name=f"drone_{i}_px4_bridge",
                 parameters=[sim_time, {"drone_id": i, "px4_ns": f"{arg('px4_ns_prefix')}{i}",
                                        "spawn_enu": list(spawn), "takeoff_height": takeoff_height,
                                        "camera_offset_frd": camera_offset_frd}],
                 remappings=[("odom", odom), ("camera_pose", camera_pose), ("position_cmd", position_cmd),
                             ("takeoff_done", drone_topic(i, "takeoff_done"))]),
        ]
        planner = [
            _node(package="ego_planner", executable="ego_planner_node", name=f"drone_{i}_ego_planner_node",
                 remappings=[
                     # advanced_param.launch.py:95-112
                     ("odom_world", odom),
                     ("planning/bspline", bspline),
                     ("planning/data_display", drone_topic(i, "planning/data_display")),
                     ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
                     ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
                     ("goal_point", drone_topic(i, "plan_vis/goal_point")),
                     ("global_list", drone_topic(i, "plan_vis/global_list")),
                     ("init_list", drone_topic(i, "plan_vis/init_list")),
                     ("optimal_list", drone_topic(i, "plan_vis/optimal_list")),
                     ("a_star_list", drone_topic(i, "plan_vis/a_star_list")),
                     ("grid_map/odom", odom),
                     ("grid_map/cloud", drone_topic(i, "pcl_render_node/cloud")),
                     ("grid_map/pose", camera_pose),
                     ("grid_map/depth", depth),
                     ("grid_map/occupancy_inflate", drone_topic(i, "grid/grid_map/occupancy_inflate")),
                     ("grid_map/occupancy", drone_topic(i, "grid/grid_map/occupancy")),
                 ],
                 parameters=[sim_time, {
                     # advanced_param.launch.py:115-195, upstream's values but where a comment says otherwise.
                     "fsm/flight_type": PRESET_TARGET,
                     "fsm/thresh_replan_time": 1.0,
                     "fsm/thresh_no_replan_meter": NEXT_WAYPOINT_M,
                     "fsm/planning_horizon": PLANNING_HORIZON_M,
                     "fsm/planning_horizen_time": 3.0,
                     "fsm/emergency_time": 1.0,
                     "fsm/realworld_experiment": True,  # wait for start_swarm's trigger instead of planning on the ground
                     "fsm/fail_safe": True,
                     "fsm/waypoint_num": len(route),
                     **{f"fsm/waypoint{k}_{axis}": w[n] for k, w in enumerate(route) for n, axis in enumerate("xyz")},
                     "grid_map/resolution": GRID_RESOLUTION_M,
                     "grid_map/map_size_x": map_size[0],
                     "grid_map/map_size_y": map_size[1],
                     "grid_map/map_size_z": map_size[2],
                     "grid_map/local_update_range_x": LOCAL_UPDATE_M,
                     "grid_map/local_update_range_y": LOCAL_UPDATE_M,
                     "grid_map/local_update_range_z": 4.5,
                     "grid_map/obstacles_inflation": OBSTACLES_INFLATION_M,  # upstream 0.099
                     "grid_map/local_map_margin": 10,
                     "grid_map/ground_height": -0.01,
                     "grid_map/cx": cx,  # upstream: a RealSense's calibration
                     "grid_map/cy": cy,
                     "grid_map/fx": fx,
                     "grid_map/fy": fy,
                     "grid_map/use_depth_filter": True,
                     "grid_map/odom_depth_timeout": ODOM_DEPTH_TIMEOUT_S,
                     "grid_map/depth_filter_tolerance": 0.15,
                     "grid_map/depth_filter_maxdist": RAY_LENGTH_M,  # farther depth becomes a free ray to the ray length (grid_map.cpp:335)
                     "grid_map/depth_filter_mindist": 0.2,
                     "grid_map/depth_filter_margin": 2,
                     "grid_map/k_depth_scaling_factor": 1000.0,
                     "grid_map/skip_pixel": 2,
                     "grid_map/p_hit": 0.65,
                     "grid_map/p_miss": 0.35,
                     "grid_map/p_min": 0.12,
                     "grid_map/p_max": 0.90,
                     "grid_map/p_occ": 0.80,
                     "grid_map/min_ray_length": 0.1,
                     "grid_map/max_ray_length": RAY_LENGTH_M,
                     "grid_map/virtual_ceil_height": ceiling,  # upstream 2.9
                     "grid_map/visualization_truncate_height": ceiling,  # upstream 1.8
                     "grid_map/show_occ_time": False,
                     "grid_map/pose_type": POSE_STAMPED,
                     "grid_map/frame_id": "world",
                     "manager/max_vel": MAX_VEL_MS,
                     "manager/max_acc": MAX_ACC_MS2,
                     "manager/max_jerk": MAX_JERK_MS3,
                     "manager/control_points_distance": 0.4,
                     "manager/feasibility_tolerance": 0.05,
                     "manager/planning_horizon": PLANNING_HORIZON_M,
                     "manager/use_distinctive_trajs": True,
                     "manager/drone_id": i,
                     "optimization/lambda_smooth": 1.0,
                     "optimization/lambda_collision": 0.5,
                     "optimization/lambda_feasibility": LAMBDA_FEASIBILITY,
                     "optimization/lambda_fitness": 1.0,
                     "optimization/dist0": 0.5,
                     "optimization/swarm_clearance": SWARM_CLEARANCE_M,  # upstream 0.5
                     "optimization/max_vel": MAX_VEL_MS,
                     "optimization/max_acc": MAX_ACC_MS2,
                     "bspline/limit_vel": MAX_VEL_MS,
                     "bspline/limit_acc": MAX_ACC_MS2,
                     "bspline/limit_ratio": 1.1,
                 }]),
            _node(package="ego_planner", executable="traj_server", name=f"drone_{i}_traj_server",
                 remappings=[("position_cmd", position_cmd), ("planning/bspline", bspline)],
                 parameters=[sim_time, {"traj_server/time_forward": TRAJ_TIME_FORWARD_S}]),
        ]
        px4_bridge = bridges[1]
        actions += bridges
        # Planner after takeoff: a planner up before PX4's odometry flags it lost for good (grid_map.cpp:716-724, :199).
        actions.append(RegisterEventHandler(OnProcessIO(
            target_action=px4_bridge,
            on_stderr=lambda event, planner=planner: planner if b"takeoff done" in event.text else None)))
    return actions


def generate_launch_description():
    share = get_package_share_directory("pterosim_ego")
    return LaunchDescription([
        DeclareLaunchArgument("spawns", description="per drone x,y,z m in the world ENU frame where PteroSim placed it, "
                                                    "';'-separated, drone 0 first"),
        DeclareLaunchArgument("goals", description="per drone waypoints x,y,z m through the forest, / between waypoints, ; between drones"),
        DeclareLaunchArgument("raw_stream_port", description="the depth cameras' raw_stream_port (no default there)"),
        DeclareLaunchArgument("control", description="PteroSim's control API, host:port, whose clock is /clock"),
        DeclareLaunchArgument("raw_stream_host", default_value="127.0.0.1",
                              description="the sensors' raw_stream_host; mirrored WSL reaches Windows on loopback"),
        DeclareLaunchArgument("px4_ns_prefix", default_value="x500_",
                              description="PX4_UXRCE_DDS_NS of instance i is <prefix><i>"),
        DeclareLaunchArgument("takeoff_height", default_value="2.0", description="m above the spawn; chosen"),
        DeclareLaunchArgument("ceiling", default_value="4.0",
                              description="m, ego-planner's virtual ceiling; chosen: under a forest canopy"),
        DeclareLaunchArgument("map_size", default_value="100,40,5",
                              description="x,y,z m of each planner's map, centred on the world origin; ~15 B a "
                                          "0.1 m voxel, so 100x40x5 is 300 MB per drone"),
        DeclareLaunchArgument("depth_width", default_value="640", description="the sensor's image_width"),
        DeclareLaunchArgument("depth_height", default_value="480", description="the sensor's image_height"),
        DeclareLaunchArgument("depth_fov_deg", default_value="90.0", description="the sensor's field_of_view"),
        DeclareLaunchArgument("camera_offset_frd", default_value="0.15,0.0,0.0",
                              description="the depth camera's mount in the body, x,y,z m FRD, level and forward"),
        # Mirrored WSL loses UDP datagrams over 1472 B to 127.0.0.1, which kills Fast DDS discovery between local nodes.
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", os.path.join(share, "config", "fastdds_wsl.xml")),
        OpaqueFunction(function=_drones),
    ])
