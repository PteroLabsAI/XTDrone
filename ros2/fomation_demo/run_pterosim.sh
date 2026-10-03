#!/bin/bash
# The formation against PteroSim instead of Gazebo: PX4 SITL per aircraft on PX4's own ROS 2 interface (one
# Micro XRCE-DDS agent for all), a communication node per VEHICLES_PER_BRIDGE of them, then the leader and the
# followers. PteroSim runs first, with the fleet from spawn_pterosim.py.
#
#   ./run_pterosim.sh <uav_type> <uav_num> <px4_root> <px4_airframe> <px4_ros_ws>
#   ./run_pterosim.sh iris 6 ~/Documents/PX4-Autopilot 22100 ~/px4_ros_ws
#
# <px4_ros_ws> holds px4_msgs built from the same PX4 tree (Tools/copy_to_ros_ws.sh, colcon build).
# Logs go to ~/formation_run/<time>/. Ctrl+C, or any process exiting, stops everything.
set -e  # not -u: ROS's setup.bash reads unset variables
[ $# -eq 5 ] || { sed -n 6p "$0"; exit 1; }
TYPE=$1 N=$2 PX4_BUILD=$(realpath "$3")/build/px4_sitl_default AIRFRAME=$4 PX4_ROS_WS=$(realpath "$5")
XRCE_AGENT_PORT=8888  # rcS's default for uxrce_dds_client
HERE=$(cd "$(dirname "$0")" && pwd)
ls "$PX4_BUILD"/etc/init.d-posix/airframes/"$AIRFRAME"_* >/dev/null
LOG=~/formation_run/$(date +%H%M%S)
mkdir -p "$LOG" && ln -sfn "$LOG" ~/formation_run/latest

source /opt/ros/humble/setup.bash
source "$PX4_ROS_WS/install/setup.bash"
# The ROS nodes' transport; the agent's participants keep Fast DDS's built-in ones (PX4 sends it a bare profile).
export FASTRTPS_DEFAULT_PROFILES_FILE=$HERE/fastdds_wsl.xml
export PYTHONUNBUFFERED=1
# Every job is the process itself (px4 is exec'd), so this reaches all of them and nothing else.
trap 'kill $(jobs -p) 2>/dev/null || true; wait' EXIT

# A copy of etc without the onboard MAVLink link: it fed MAVROS, nothing listens on it now. The PX4 tree stays untouched.
ETC=$LOG/etc
cp -r "$PX4_BUILD/etc" "$ETC"
RC=$ETC/init.d-posix/px4-rc.mavlink
ONBOARD_LINK='^mavlink start .* -m onboard -o \$udp_offboard_port_remote$'
grep -q "$ONBOARD_LINK" "$RC" || { echo "$RC no longer has the line this script removes; check px4-rc.mavlink"; exit 1; }
sed -i "/$ONBOARD_LINK/d" "$RC"

# Started before the PX4s, so every client finds it on its first attempt.
MicroXRCEAgent udp4 -p $XRCE_AGENT_PORT >"$LOG/agent.log" 2>&1 &
for ((i = 0; i < N; i++)); do
    root=$PX4_BUILD/rootfs/$i
    # A parameter store left by another airframe would leak its gains into this one.
    mkdir -p "$root" && rm -f "$root"/parameters*.bson
    # No PX4_SIM_MODEL: the simulator is PteroSim, which dials nothing; PX4 instance i connects to its
    # HIL server on 4560 + i, which mirrored WSL reaches on loopback.
    # UXRCE_DDS_SYNCT=0: with time sync on, half of 18 clients never finished their handshake (stale replies, silent).
    (cd "$root" && exec env -u PX4_SIM_MODEL PX4_SIM_HOSTNAME=127.0.0.1 PX4_SYS_AUTOSTART=$AIRFRAME \
        PX4_UXRCE_DDS_NS=${TYPE}_$i PX4_UXRCE_DDS_PORT=$XRCE_AGENT_PORT PX4_PARAM_UXRCE_DDS_SYNCT=0 \
        "$PX4_BUILD/bin/px4" -i $i -d "$ETC" </dev/null >"$LOG/px4_$i.log" 2>&1) &
done
# Measured on MAVROS-fed bridges: 18 vehicles in one process filled its core (1.02, the GIL); three of six cost 0.74.
VEHICLES_PER_BRIDGE=6
for ((first = 0; first < N; first += VEHICLES_PER_BRIDGE)); do
    last=$((first + VEHICLES_PER_BRIDGE < N ? first + VEHICLES_PER_BRIDGE - 1 : N - 1))
    python3 "$HERE/../communication/multirotor_communication.py" "$TYPE" $(seq $first $last) >"$LOG/communication_$first.log" 2>&1 &
done

cd "$HERE"
python3 leader.py "$TYPE" "$N" >"$LOG/leader.log" 2>&1 &
python3 follower.py "$TYPE" "$N" >"$LOG/follower.log" 2>&1 &
echo "running; logs in $LOG"
wait -n -p gone || true
echo "process $gone exited; stopping everything, see $LOG"
exit 1
