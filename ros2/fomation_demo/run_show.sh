#!/bin/bash
# The flight for the camera against a running PteroSim with the fleet spawned (spawn_pterosim.py): run_pterosim.sh in
# tmux, fly_show.py in the foreground, teardown. PteroSim's scripts/filmmaking/record_swarm.py runs this through wsl.
#
#   ./run_show.sh <ros2 tree> <uav_num> <px4_root> <px4_ros_ws>
#   ./run_show.sh ~/formation_run/ros2 18 ~/Documents/PX4-Autopilot ~/px4_ros_ws
[ $# -eq 4 ] || { sed -n 5p "$0"; exit 1; }
TREE=$1 N=$2 PX4_ROOT=$3 PX4_ROS_WS=$4
D=$TREE/fomation_demo
AIRFRAME=22100           # PX4's x500 airframe for PteroSim
NEW_LOG_WAIT_S=60        # chosen: run_pterosim.sh makes its log folder within a second or two
source /opt/ros/humble/setup.bash
source "$PX4_ROS_WS/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE=$D/fastdds_wsl.xml ROS_DOMAIN_ID=42 PYTHONUNBUFFERED=1

# run_pterosim.sh points ~/formation_run/latest at its new log folder; the old link must not be mistaken for it.
OLD=$(readlink -f ~/formation_run/latest)
tmux new-session -d -s formation -x 200 -y 50 "cd $D && bash run_pterosim.sh iris $N $PX4_ROOT $AIRFRAME $PX4_ROS_WS; sleep 5"
for _ in $(seq $NEW_LOG_WAIT_S); do L=$(readlink -f ~/formation_run/latest); [ "$L" != "$OLD" ] && break; sleep 1; done
[ "$L" = "$OLD" ] && { echo "[show] run_pterosim.sh made no new log folder"; tmux kill-session -t formation; exit 1; }
echo "[show] logs $L"
(cd $D && python3 fly_show.py iris $N 2>&1 | tee "$L/fly_show.log")
tmux kill-session -t formation
echo "[show] done"
