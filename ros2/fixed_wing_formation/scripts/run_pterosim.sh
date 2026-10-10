#!/bin/bash
# XTDrone's fixed-wing VTOL formation against PteroSim: three PX4 SITL standard_vtol on PX4's own ROS 2 interface
# (one Micro XRCE-DDS agent for all), the two fw_formation_follower nodes, then fly_fw_formation.py with the given
# arguments. PteroSim runs first, with the three aircraft from spawn_pterosim.py.
#
#   ./run_pterosim.sh <px4_root> <px4_ros_ws> <fly_fw_formation.py arguments>
#   ./run_pterosim.sh ~/Documents/PX4-Autopilot ~/px4_ros_ws --alt 120 --route 0,2500 1500,2500 --figures 3,40 1,40 2,40
#
# <px4_ros_ws> holds px4_msgs from the same PX4 tree and this package (colcon build). Logs, and each PX4's rootfs with
# its parameters and dataman, go to ~/fw_formation_run/<time>/. Everything stops when fly_fw_formation.py ends.
set -e  # not -u: ROS's setup.bash reads unset variables
[ $# -ge 3 ] || { sed -n 6p "$0"; exit 1; }
PX4_BUILD=$(realpath "$1")/build/px4_sitl_default PX4_ROS_WS=$(realpath "$2")
shift 2
AIRFRAME=22003             # standard_vtol for PteroSim (PX4's 4004 without the Gazebo lines)
N=3                        # the leader and the law's two followers
XRCE_AGENT_PORT=8888       # rcS's default for uxrce_dds_client
GCS_PORT_BASE=18570        # px4-rc.mavlink:11, the UDP port PX4 instance i binds for fly_fw_formation.py
HOLD_ON_OFFBOARD_LOSS=5    # COM_OBL_RC_ACT: Hold (an FW loiter); its default, Position, falls through to RTL without RC
HERE=$(cd "$(dirname "$0")" && pwd)
ls "$PX4_BUILD"/etc/init.d-posix/airframes/"$AIRFRAME"_* >/dev/null

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

source /opt/ros/humble/setup.bash
source "$PX4_ROS_WS/install/setup.bash"
# The ROS nodes' transport (mirrored WSL loses Fast DDS's large discovery datagrams); the agent keeps its own.
export FASTRTPS_DEFAULT_PROFILES_FILE=$HERE/../../fomation_demo/fastdds_wsl.xml
export PYTHONUNBUFFERED=1
trap 'kill $(jobs -p) 2>/dev/null || true; wait' EXIT

# A copy of etc without the onboard MAVLink link: nothing listens on it here. The PX4 tree stays untouched.
ETC=$LOG/etc
cp -r "$PX4_BUILD/etc" "$ETC"
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
    (cd "$root" && exec env -u PX4_SIM_MODEL PX4_SIM_HOSTNAME=127.0.0.1 PX4_SYS_AUTOSTART=$AIRFRAME \
        PX4_UXRCE_DDS_NS=standard_vtol_$i PX4_UXRCE_DDS_PORT=$XRCE_AGENT_PORT PX4_PARAM_UXRCE_DDS_SYNCT=0 \
        PX4_PARAM_COM_OBL_RC_ACT=$HOLD_ON_OFFBOARD_LOSS \
        "$PX4_BUILD/bin/px4" -i $i -d "$ETC" </dev/null >"$LOG/px4_$i.log" 2>&1) &
done

ros2 launch fixed_wing_formation fw_formation.launch.py leader:=standard_vtol_0 \
    follower1:=standard_vtol_1 follower2:=standard_vtol_2 formation_type:=3 >"$LOG/followers.log" 2>&1 &
python3 "$HERE/fly_fw_formation.py" "$@" 2>&1 | tee "$LOG/fly.log"
status=${PIPESTATUS[0]}
echo "fly_fw_formation.py exited $status; logs in $LOG"
exit $status
