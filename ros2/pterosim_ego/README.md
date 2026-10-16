# pterosim_ego

N PteroSim x500s fly line abreast through a forest, each on its own PX4 SITL in lockstep with PteroSim, planned by
ego-planner-swarm (`ros2_version`, the `ros2/ego-planner-swarm` submodule at 23a8d5a) from each drone's own depth camera.
This package is the ROS 2 Humble bridge between the three; `scripts/` holds what runs a flight.

```
PteroSim depth camera --PTF1/TCP--> depth_bridge --Image 16UC1--> ego_planner_node (grid map)
PteroSim control API --GetSimulationClock--> pterosim_clock --/clock--> every node (use_sim_time)
PX4 SITL --uXRCE-DDS--> px4_bridge --Odometry, camera PoseStamped--> ego_planner_node --B-spline--> traj_server
traj_server --PositionCommand--> px4_bridge --OffboardControlMode, TrajectorySetpoint, VehicleCommand--> PX4
start_swarm --/traj_start_trigger--> every ego_planner_node, once every drone is up
```

Drone `i` is PteroSim instance `i`, PX4 instance `i` (uXRCE namespace `x500_<i>`, MAV_SYS_ID `i+1`, HIL port
`4560+i`) and ego-planner drone `i`. Ids must run 0..N-1: planner `i` starts only after planner `i-1` has published
its trajectory (`ego_replan_fsm.cpp:77`).

The parameters below are set for a 90 m pine corridor of the Mediterranean Coast map (Fab).

## What it needs

Windows:

- PteroSim with a forest map, playing with PteroSim's game mode (World Settings > GameMode Override:
  `PteroSimGameMode`) and a license tier that allows N aircraft. Its scripting server listens on `127.0.0.1:10010`.
- Python 3.10+ with PteroSim's Python SDK, which ships inside the PteroSim release:
  `pip install <PteroSim>/PteroSim/Plugins/PteroSimScripting/SDK/python`.

WSL2 (Ubuntu 22.04, mirrored networking, so WSL reaches PteroSim on `127.0.0.1`):

- ROS 2 Humble, tmux, and the Micro XRCE-DDS Agent (`MicroXRCEAgent` on `PATH`).
- A PX4-Autopilot tree with PteroSim's x500 airframe: `x500/firmwares/px4_x500` of PteroLabsAI/PteroSimAircrafts as
  `ROMFS/px4fmu_common/init.d-posix/airframes/22100_pterosim_x500`, listed in that folder's `CMakeLists.txt`.
- `px4_msgs` built from the same PX4 tree in its own workspace (`Tools/copy_to_ros_ws.sh`, `colcon build`): the
  message versions must match the firmware.
- PteroSim's Python SDK, for `pterosim_clock`: `pip install --user` the same release folder, through `/mnt/c/...`.

`ros2/Dockerfile` builds this package and the patched planner (it applies the patch below and fails if it does not
apply), but the image has no PX4, `px4_msgs`, agent or SDK: flying needs the WSL setup described here.

## Build (WSL)

ego-planner-swarm gets one patch first, `patches/ego-planner-swarm-sim-clock.patch` (see Time):

```bash
git clone https://github.com/PteroLabsAI/XTDrone.git ~/XTDrone
git -C ~/XTDrone submodule update --init ros2/ego-planner-swarm
git -C ~/XTDrone/ros2/ego-planner-swarm apply ../pterosim_ego/patches/ego-planner-swarm-sim-clock.patch

mkdir -p ~/ego_ws/src && ln -s ~/XTDrone/ros2/ego-planner-swarm ~/XTDrone/ros2/pterosim_ego ~/ego_ws/src/
cd ~/ego_ws
source /opt/ros/humble/setup.bash
grep -v -e '^#' -e '^$' ~/XTDrone/ros2/apt.txt | xargs sudo apt-get install -y
rosdep install --from-paths src --ignore-src -y --skip-keys "PCL Eigen3 OpenCV Boost Armadillo dynamic_reconfigure tf px4_msgs"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then PX4, once per tree, into its own build dir (see Parameters, trimmed DDS topics):

```bash
~/XTDrone/ros2/pterosim_ego/scripts/wsl/px4_build.sh ~/PX4-Autopilot
```

## Run

`scripts/windows/` runs on Windows, next to PteroSim; `scripts/wsl/` runs in WSL. `swarm_forest.py` calls the WSL side
itself through `wsl.exe`.

| Script | Side | |
|---|---|---|
| `swarm_forest.py` | Windows | spawns the drones, runs `run_swarm.sh`, waits for the goals |
| `px4_build.sh` | WSL | PX4 SITL with the trimmed topic list, `build/px4_sitl_ego` |
| `run_swarm.sh` | WSL | in tmux session `ego`: the launch, then the PX4s (`px4_swarm.sh`), then `start_swarm`'s trigger |
| `stop.sh` | WSL | ends the PX4s, the launch and its nodes |

A flight, with the map playing:

```powershell
python swarm_forest.py --start 466,334 --heading 150 --length 90 --drones 8 --fly-height 2.5 `
    --distro Ubuntu-22.04 --px4 /home/<you>/PX4-Autopilot --px4-ros-ws /home/<you>/px4_ros_ws --ego-ws /home/<you>/ego_ws
```

`--start` is the middle of the starting line and `--heading` the direction of flight, both in UE (metres, yaw); the
values above start the 90 m pine corridor of the Mediterranean Coast map. The script removes every aircraft in the
scene and spawns N x500s 5 m apart across the heading, each with a 320x240 depth camera and GPS noise off. A spawn
lands on the surface under it, so each drone's reported height is its ground. Its goal is the point `--length` ahead
of its spawn, `--fly-height` above that ground, which is also its takeoff height; the planners find their way around
the trees from their own depth cameras. The script starts the simulation at time scale 1, then `run_swarm.sh`, which
returns once the planners are triggered; then it prints the real-time factor and each drone's distance to its goal
every 10 s until all are within 3 m, and fails if the swarm averages under 1 m/s of simulation time. The swarm keeps
flying afterwards: `scripts/wsl/stop.sh` ends it. Logs are in WSL `~/ego_forest_run/latest/` (`launch.sh` holds the
exact launch command).

## Parameters, and why

Set for a line of 8 drones on this corridor. Each value is a named constant in the code, and its comment gives the
same reason.

| What | Value | Why |
|---|---|---|
| `manager/max_vel` (launch `MAX_VEL_MS`) | 5 m/s | PX4 lags the plan's turns between trunks more the faster it flies; the limit bounds that lag |
| `grid_map/obstacles_inflation` | 0.71 m | prop tip radius 0.41 m plus a 0.3 m tracking margin; a wider margin closes the gaps between this forest's trunks and puts waypoints inside crowns, a narrower one lets the props reach branches |
| `grid_map/max_ray_length`, `depth_filter_maxdist` | 8 m | stopping from 5 m/s at the 4 m/s² limit takes 3.1 m plus the replan, and 8 m of range leaves room for both; depth past `depth_filter_maxdist` becomes a free ray (`grid_map.cpp:335`), so both are the same |
| px4_bridge brake-and-park | | on an emergency stop ego-planner commands a standstill where the drone was; flown as a position target from speed, PX4 overshoots it and then flies back through space the camera has not mapped. The bridge answers a command that drops from over 1 m/s to zero with a zero-velocity setpoint, holds where the drone settles (under 0.1 m/s), and follows again once the command moves |
| px4_bridge yaw rate | 45 deg/s at most | ego-planner aims the nose 1 s ahead along its weaving path, which swings the depth camera from side to side; the limit is chosen |
| `optimization/lambda_feasibility` | 1.0 (upstream 0.1) | weights the plans' feasibility cost up, toward the 4 m/s² limit; chosen |
| `grid_map/odom_depth_timeout` | 3 s (upstream 1 s) | ego-planner latches EMERGENCY_STOP for good after a longer gap (`grid_map.cpp:716-724`); a swarm's start, every PX4 and planner coming up at once, can hold odometry back for more than 1 s |
| planner start | after px4_bridge's takeoff | a planner up before PX4's odometry flags it lost for good (`grid_map.cpp:716-724`, `:199`) |
| depth camera | 320x240 at 30 Hz | 640x480 16-bit depth at 30 Hz is 18 MB/s per drone in 1400-byte datagrams; 320x240 is a quarter of that |
| GPS noise | off | a swarm flies on RTK; independent 1.7 m errors put neighbours 5 m apart into each other |
| `SENS_BAR_AUTOCAL` | 0 | at spin-up PX4 offsets the barometer onto the GPS altitude, which copies the GPS error into the height estimate |
| `EKF2_HGT_REF` | 0 (baro) | the GPS height reference carries the GPS altitude noise into a hover a few metres above the ground |
| `EKF2_GPS_CTRL` | 5 (no GPS altitude) | GPS altitude still pulls a baro-referenced estimate; 5 uses GPS for position only |
| `UXRCE_DDS_SYNCT` | 0 | with time sync on, a share of the clients never finish their handshake |
| PX4 start | 1 s apart | started at once, a client can time out creating its `trajectory_setpoint` reader, which PX4 never retries |
| trimmed DDS topics (`scripts/wsl/dds_topics.yaml`) | odometry at 50 Hz and command acks out; offboard mode, setpoint and command in | px4_bridge uses these four; PX4's ~30 default topics cost the agents and DDS discovery for nothing |
| uXRCE agents | one per 2 drones | an agent serves all its clients on one thread |
| `grid_map/resolution` | 0.15 m | each planner allocates its whole map; eight at 0.1 m on this corridor's map would hold 2.8 GB each |

The other launch values are upstream's (`advanced_param.launch.py`) unless the launch says why not.

## Time

No node here reads the wall clock. PX4 under lockstep sets its clock from HIL_SENSOR `time_usec`, which PteroSim fills
with FDM time, and the depth camera stamps each frame with the FDM time of the state it shows. So:

- `/clock`: `pterosim_clock`, PteroSim's `GetSimulationClock` (the physics time of the aircraft furthest along).
- Depth images: the frame's `sim_us`.
- Odometry and camera pose: `VehicleOdometry.timestamp_sample`.
- PX4-bound messages: the latest `VehicleOdometry.timestamp`, so OFFBOARD lapses (COM_OF_LOSS_T) when px4_bridge
  stops hearing PX4. It runs on odometry callbacks, never on a timer.

Upstream ego-planner reads plan times from `rclcpp::Clock()`, the system clock, at 23 call sites in the FSM, the
planner manager and the optimizer, from an explicit `RCL_SYSTEM_TIME` clock in the FSM's swarm check, and from an
unattached `rclcpp::Clock(RCL_ROS_TIME)` in `traj_server`, whatever `use_sim_time` says. Under lockstep below real
time the plans then run ahead of the drones. `patches/ego-planner-swarm-sim-clock.patch` adds `bspline_opt/ego_clock.h`, one clock per process that `ego_planner_node`'s and `traj_server`'s `main()` set to their
node's, and reads every plan time from it, so the planners run on `/clock`. ego-planner's visualization markers keep
the system clock.

## Nodes

### pterosim_clock

Parameter `control` (string, `host:port` of PteroSim's scripting server). Publishes `/clock` once per PteroSim frame.
PteroSim's clock restarts from 0 on every Start, which a running ROS graph cannot follow back: restart the launch
(`run_swarm.sh` does) after a Stop and Start. Its exit ends the launch.

### depth_bridge

Reads PTF1 frames from `host:raw_stream_port + 2 * drone_id` and publishes them as ego-planner's depth image. Ends with
an error when the stream closes, or when a frame is not depth, is larger than 640x480, or disagrees with the
intrinsics or mount given below.

| Parameter | Type | |
|---|---|---|
| `drone_id` | int | PteroSim instance id |
| `host`, `raw_stream_port` | string, int | the sensor's `raw_stream_host` and `raw_stream_port` |
| `fx`, `fy`, `cx`, `cy` | double | what the planner was given; each frame's own must match within 0.01 px |
| `camera_offset_frd` | double[3] | the mount; each frame's camera position must match within 1 mm, rotation level |

| Topic | Type | |
|---|---|---|
| `depth` -> `/drone_<i>_pcl_render_node/depth` | sensor_msgs/Image | 16UC1 mm, stamp = frame's `sim_us` |

### px4_bridge

| Parameter | Type | |
|---|---|---|
| `drone_id` | int | PX4 instance; commands go to system `drone_id + 1` |
| `px4_ns` | string | PX4_UXRCE_DDS_NS of that instance |
| `spawn_enu` | double[3] | where PX4's local origin (the boot pose) sits in the swarm world, ENU m |
| `takeoff_height` | double | m climbed before ego-planner takes over |
| `camera_offset_frd` | double[3] | the depth camera in the body, FRD m (level, forward) |

| Topic | Type | Direction |
|---|---|---|
| `/<ns>/fmu/out/vehicle_odometry` | px4_msgs/VehicleOdometry | in, best effort |
| `/<ns>/fmu/out/vehicle_command_ack` | px4_msgs/VehicleCommandAck | in, best effort |
| `/<ns>/fmu/in/offboard_control_mode` | px4_msgs/OffboardControlMode | out, every odometry |
| `/<ns>/fmu/in/trajectory_setpoint` | px4_msgs/TrajectorySetpoint | out, every odometry |
| `/<ns>/fmu/in/vehicle_command` | px4_msgs/VehicleCommand | out: OFFBOARD, then ARM |
| `odom` -> `/drone_<i>_visual_slam/odom` | nav_msgs/Odometry | out |
| `camera_pose` -> `/drone_<i>_pcl_render_node/camera_pose` | geometry_msgs/PoseStamped | out |
| `position_cmd` -> `/drone_<i>_planning/pos_cmd` | quadrotor_msgs/PositionCommand | in |
| `takeoff_done` -> `/drone_<i>_takeoff_done` | std_msgs/Empty, transient local | out, once |

It streams setpoints for 1 s, switches PX4 to OFFBOARD, arms (PX4 refuses for the first ~20 s after boot) and climbs
`takeoff_height`. An unanswered command goes again every 0.5 s and the node fails after 5 sends; a refused one goes
again every 2 s and the node fails after 30 refusals. After takeoff it follows the latest position command, with the
brake-and-park and yaw limit above; if none has come for 0.5 s (PX4 time) it holds the last one.

### start_swarm

Parameter `drones` (int). Waits for every `/drone_<i>_takeoff_done` and for every planner to subscribe, or sends for the
drones that are up 30 s (sim time) after the first takeoff; publishes `/traj_start_trigger` (geometry_msgs/PoseStamped)
once, and fails if a planner does not acknowledge it within 5 s.

## Launch arguments

| Argument | Default | |
|---|---|---|
| `spawns` | required | per drone `x,y,z` m in the swarm's ENU world, `;` between drones, drone 0 first |
| `goals` | required | per drone waypoints `x,y,z` m, `/` between waypoints, `;` between drones, same order |
| `raw_stream_port` | required | the depth cameras' `raw_stream_port` |
| `control` | required | PteroSim's scripting server, `host:port`, whose clock is `/clock` |
| `raw_stream_host` | `127.0.0.1` | mirrored WSL reaches Windows on loopback |
| `px4_ns_prefix` | `x500_` | namespace of PX4 instance `i` is `<prefix><i>` |
| `takeoff_height` | `2.0` | m |
| `ceiling` | `4.0` | ego-planner's virtual ceiling, m (upstream 2.9) |
| `map_size` | `100,40,5` | each planner's map, m, centred on the world origin |
| `depth_width`, `depth_height`, `depth_fov_deg` | `640`, `480`, `90.0` | the sensor's `image_width`, `image_height`, `field_of_view` |
| `camera_offset_frd` | `0.15,0.0,0.0` | the depth camera's mount, FRD m |

The world origin is the middle of the map: ego-planner's map spans `-x/2..x/2`, `-y/2..y/2` and
`ground_height..ground_height+z` (`grid_map.cpp:92`); the launch refuses spawns or goals outside it.

## ego-planner's topics (23a8d5a)

| What | Name | Source |
|---|---|---|
| depth in | `grid_map/depth` -> `/drone_<i>_pcl_render_node/depth`, 16UC1 mm (32FC1 converted) | `advanced_param.launch.py:111`, `run_in_sim.launch.py:53`, `grid_map.cpp:139, :765` |
| camera pose in (pose_type 1) | `grid_map/pose` -> `/drone_<i>_pcl_render_node/camera_pose`, ApproximateTime with depth | `advanced_param.launch.py:110`, `grid_map.h:233`, `grid_map.cpp:146-155` |
| odometry in | `odom_world`, `grid_map/odom` -> `/drone_<i>_visual_slam/odom` | `advanced_param.launch.py:96, :108`, `swarm.launch.py:17` |
| position command out | `/position_cmd` -> `/drone_<i>_planning/pos_cmd` | `traj_server.cpp:244`, `run_in_sim.launch.py:94` |
| start, flight_type 2 | `/traj_start_trigger`, one for all drones | `ego_replan_fsm.cpp:127, :145` |
| swarm | `/broadcast_bspline`, `/drone_<i>_planning/swarm_trajs` | `advanced_param.launch.py:99-100`, `ego_replan_fsm.cpp:79, :98` |

## Frames

PX4 gives NED world / FRD body; ROS and ego-planner use ENU world / FLU body; ego-planner projects depth in the
optical frame (x image right, y image down, z forward).

- Position: world ENU = `spawn_enu` + (E, N, -D) of PX4's local position. Each PX4 has its own local origin, where it
  booted; `spawn_enu` puts them in one world.
- Attitude: q_ENU<-FLU = q_ENU<-NED * q_NED<-FRD * q_FRD<-FLU, with q_ENU<-NED = (0, sqrt(1/2), sqrt(1/2), 0) and
  q_FRD<-FLU = (0, 1, 0, 0) (w, x, y, z).
- Linear velocity in `nav_msgs/Odometry.twist`: world ENU, because ego-planner reads it as world velocity
  (`ego_replan_fsm.cpp:255-257`). Angular velocity: body FLU.
- Camera pose: position = body + R_body * (x, -y, -z) of `camera_offset_frd`; orientation = q_body * q_FLU<-optical,
  q_FLU<-optical = (0.5, -0.5, 0.5, -0.5), the rotation ego-planner itself uses (`grid_map.cpp:131`).
- Setpoint: local NED = (N, E, -U) of (world ENU - `spawn_enu`); velocity and acceleration the same swap; yaw_NED =
  pi/2 - yaw_ENU; yawspeed_NED = -yaw_dot_ENU.
- Depth: float32 m -> uint16 mm; +inf (nothing within reach), nan, <= 0 or past 65.535 m -> 0, which ego-planner
  takes as free out to `max_ray_length` (`grid_map.cpp:327`).

## Limits

- PteroSim 0.4.0 steps each aircraft when its own PX4 answers, so a drone whose PX4 falls behind trails `/clock` (the
  aircraft furthest along). Its planner then plans ahead of it, logs "Time difference is too large" in the swarm
  exchange, and past 3 s stops on a false odometry timeout. Later PteroSim versions step the whole world only after
  every aircraft's autopilot has answered.
- PX4 lags the plan in fast turns; `MAX_VEL_MS` is the knob.
- Each goal sits `--fly-height` above its own spawn's ground. Ground that rises more than that toward the goals puts
  them underground: start on level ground or fly higher, under the launch's ceiling.
- 640x480 at most: ego-planner's projection buffer is sized for it and never checks (`grid_map.cpp:128`).
- The depth camera must be mounted level and forward; only its offset is configurable.
- One route per drone. `/traj_start_trigger` reaches every planner; after its goal a planner waits for the next one.
- A crash closes that drone's depth stream: its bridge exits and the others fly on.
- Fast DDS: the launch sets `FASTRTPS_DEFAULT_PROFILES_FILE` to `config/fastdds_wsl.xml` (UDP only, messages under
  1400 bytes), since mirrored WSL loses larger loopback datagrams. Any other ROS process on the swarm (rviz2,
  `ros2 topic`) needs the same variable.

## Tests

```bash
cd ~/ego_ws && colcon test --packages-select pterosim_ego && colcon test-result --verbose
```
