#!/bin/bash
# PteroSim VTOL formation: quadtailsitter, tiltrotor and explora.
# One Micro XRCE-DDS agent serves all three PX4 SITLs; two ROS follower nodes control instances 1 and 2.
# PteroSim must already contain the three aircraft from spawn_pterosim.py.
#
#   ./run_pterosim.sh <px4_root> <px4_ros_ws> <fly_fw_formation.py arguments>
#   ./run_pterosim.sh ~/Documents/PX4-Autopilot ~/ros2_px4_ws --alt 120 --route 0,2500 1500,2500 --figures 3,40 1,40 2,40
#
# <px4_ros_ws> holds px4_msgs from the same PX4 tree and this package (colcon build). Logs, and each PX4's rootfs with
# its parameters and dataman, go to ~/fw_formation_run/<time>/. Everything stops when fly_fw_formation.py ends.
set -e  # not -u: ROS's setup.bash reads unset variables
[ $# -ge 3 ] || { sed -n 6p "$0"; exit 1; }
PX4_BUILD=$(realpath "$1")/build/px4_sitl_default PX4_ROS_WS=$(realpath "$2")
shift 2
PTEROSIM_AIRCRAFTS=${PTEROSIM_AIRCRAFTS:-/mnt/c/Users/Yollnahkriin/Documents/Unreal_Projects/PteroSim/PteroSimAircrafts}
AIRCRAFT_NAMES=(quadtailsitter tiltrotor explora)
AIRFRAME_IDS=(22007 22009 22006)
PROFILE_FILES=(
    "$PTEROSIM_AIRCRAFTS/quadtailsitter/px4_quadtailsitter"
    "$PTEROSIM_AIRCRAFTS/tiltrotor/firmwares/px4_tiltrotor"
    "$PTEROSIM_AIRCRAFTS/explora/firmwares/px4_explora"
)
N=3                       # leader and the law's two followers
XRCE_AGENT_PORT=8888       # rcS's default for uxrce_dds_client
GCS_PORT_BASE=18570        # px4-rc.mavlink:11, the UDP port PX4 instance i binds for fly_fw_formation.py
HOLD_ON_OFFBOARD_LOSS=5    # COM_OBL_RC_ACT: Hold (an FW loiter); its default, Position, falls through to RTL without RC
HERE=$(cd "$(dirname "$0")" && pwd)
PX4_SIM_HOSTNAME=${PX4_SIM_HOSTNAME:-$(ip -4 route show default | awk 'NR==1 {print $3}')}
[ -n "$PX4_SIM_HOSTNAME" ] || { echo "could not determine the Windows host IP; set PX4_SIM_HOSTNAME"; exit 1; }
for profile in "${PROFILE_FILES[@]}"; do
    [ -f "$profile" ] || { echo "missing PteroSim PX4 profile: $profile"; exit 1; }
done

# Nothing starts while another run (another session's) holds what this one needs. TCP 4560 + i needs no check: PteroSim
# listens there on Windows and PX4 dials it (SimulatorMavlink.cpp:1170).
for pid in $(pgrep -x px4 || true); do
    [ "$(readlink -f /proc/$pid/exe)" != "$(readlink -f "$PX4_BUILD/bin/px4")" ] ||
        { echo "px4 (pid $pid) from $PX4_BUILD is already running"; exit 1; }
done
for port in $(seq $GCS_PORT_BASE $((GCS_PORT_BASE + N - 1))) $XRCE_AGENT_PORT; do
    [ -z "$(ss -Hanu "sport = :$port")" ] || { echo "UDP $port is already bound: another SITL run is up"; exit 1; }
done

LOG=~/fw_formation_run/$(date +%H%M%S)
mkdir -p "$LOG" && ln -sfn "$LOG" ~/fw_formation_run/latest

ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
[ -f "$ROS_SETUP" ] || { echo "ROS setup not found: $ROS_SETUP (set ROS_SETUP to your installed ROS 2 distro)"; exit 1; }
[ -x "$PX4_BUILD/bin/px4" ] || { echo "PX4 SITL is not built: $PX4_BUILD/bin/px4"; exit 1; }
[ -f "$PX4_ROS_WS/install/setup.bash" ] || { echo "ROS workspace is not built: $PX4_ROS_WS/install/setup.bash"; exit 1; }
source "$ROS_SETUP"
source "$PX4_ROS_WS/install/setup.bash"
ros2 pkg prefix fixed_wing_formation >/dev/null 2>&1 || {
    echo "ROS package fixed_wing_formation is not installed in $PX4_ROS_WS; add its source to that workspace and build it"
    exit 1
}
if ! command -v MicroXRCEAgent >/dev/null && [ -x "$PX4_ROS_WS/build/microxrcedds_agent/MicroXRCEAgent" ]; then
    export PATH="$PX4_ROS_WS/build/microxrcedds_agent:$PATH"
fi
command -v MicroXRCEAgent >/dev/null || { echo "MicroXRCEAgent not found in PATH or $PX4_ROS_WS/build/microxrcedds_agent"; exit 1; }
# The ROS nodes' transport (mirrored WSL loses Fast DDS's large discovery datagrams); the agent keeps its own.
export FASTRTPS_DEFAULT_PROFILES_FILE=$HERE/../../fomation_demo/fastdds_wsl.xml
export PYTHONUNBUFFERED=1
trap 'kill $(jobs -p) 2>/dev/null || true; wait' EXIT

# A copy of etc without the onboard MAVLink link: nothing listens on it here. The PX4 tree stays untouched.
ETC=$LOG/etc
cp -r "$PX4_BUILD/etc" "$ETC"
AIRFRAME_DIR=$ETC/init.d-posix/airframes
for ((i = 0; i < N; i++)); do
    target="$AIRFRAME_DIR/${AIRFRAME_IDS[$i]}_pterosim_${AIRCRAFT_NAMES[$i]}"
    # The PteroSim profiles live on a Windows checkout and may have CRLF endings;
    # PX4's /bin/sh treats the carriage return as part of each command.
    sed 's/\r$//' "${PROFILE_FILES[$i]}" > "$target"
    chmod 0755 "$target"
done
for ((i = 0; i < N; i++)); do
    test -x "$AIRFRAME_DIR/${AIRFRAME_IDS[$i]}_pterosim_${AIRCRAFT_NAMES[$i]}"
done
RC=$ETC/init.d-posix/px4-rc.mavlink
ONBOARD_LINK='^mavlink start .* -m onboard -o \$udp_offboard_port_remote$'
grep -q "$ONBOARD_LINK" "$RC" || { echo "$RC no longer has the line this script removes; check px4-rc.mavlink"; exit 1; }
sed -i "/$ONBOARD_LINK/d" "$RC"

MicroXRCEAgent udp4 -p $XRCE_AGENT_PORT >"$LOG/agent.log" 2>&1 &
for ((i = 0; i < N; i++)); do
    root=$LOG/rootfs_$i
    mkdir "$root"
    # No PX4_SIM_MODEL: PteroSim is the simulator; instance i connects to its HIL server on 4560 + i.
    # UXRCE_DDS_SYNCT=0: with time sync on, half of 18 clients never finished their handshake.
    (cd "$root" && exec env -u PX4_SIM_MODEL PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME" PX4_SYS_AUTOSTART=${AIRFRAME_IDS[$i]} \
        PX4_UXRCE_DDS_NS=${AIRCRAFT_NAMES[$i]}_$i PX4_UXRCE_DDS_PORT=$XRCE_AGENT_PORT PX4_PARAM_UXRCE_DDS_SYNCT=0 \
        PX4_PARAM_COM_OBL_RC_ACT=$HOLD_ON_OFFBOARD_LOSS \
        "$PX4_BUILD/bin/px4" -i $i -d "$ETC" </dev/null >"$LOG/px4_$i.log" 2>&1) &
done

ros2 launch fixed_wing_formation fw_formation.launch.py leader:=${AIRCRAFT_NAMES[0]}_0 \
    follower1:=${AIRCRAFT_NAMES[1]}_1 follower2:=${AIRCRAFT_NAMES[2]}_2 formation_type:=3 >"$LOG/followers.log" 2>&1 &
python3 "$HERE/fly_fw_formation.py" "$@" 2>&1 | tee "$LOG/fly.log"
status=${PIPESTATUS[0]}
echo "fly_fw_formation.py exited $status; logs in $LOG"
exit $status
