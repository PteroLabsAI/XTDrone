#!/bin/bash
# The swarm's WSL side in tmux session "ego": forest_swarm.launch.py, then the PX4s, then start_swarm's one trigger once
# every drone is up. Returns after the trigger and leaves the swarm flying; stop.sh ends it. PteroSim runs first.
#
#   ./run_swarm.sh <px4_root> <px4_ros_ws> <ego_ws> <drones> <launch arguments...>
#
# <px4_ros_ws> holds px4_msgs built from the same PX4 tree, <ego_ws> pterosim_ego and the patched ego-planner-swarm.
# Logs go to ~/ego_forest_run/<time>/.
set -e  # not -u: ROS's setup.bash reads unset variables
[ $# -ge 5 ] || { sed -n 5p "$0"; exit 1; }
PX4_ROOT=$(realpath "$1") PX4_ROS_WS=$(realpath "$2") EGO_WS=$(realpath "$3") N=$4
shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
SESSION=ego
# Chosen: the launch's agents are up within seconds, and a PX4 started before its agent waits for it.
LAUNCH_HEAD_START_S=10
# Chosen: 8 PX4 boots, ~20 s each of refused arming, and their takeoffs, with room to spare.
TRIGGER_TIMEOUT_S=240

"$HERE/stop.sh"
LOG=~/ego_forest_run/$(date +%H%M%S)
mkdir -p "$LOG" && ln -sfn "$LOG" ~/ego_forest_run/latest
ENV="source /opt/ros/humble/setup.bash && source $PX4_ROS_WS/install/setup.bash && source $EGO_WS/install/setup.bash"
echo "$ENV && exec ros2 launch pterosim_ego forest_swarm.launch.py $(printf '%q ' "$@")" > "$LOG/launch.sh"
tmux new-session -d -s $SESSION -n launch "bash $LOG/launch.sh 2>&1 | tee $LOG/launch.log"
sleep $LAUNCH_HEAD_START_S
tmux new-window -t $SESSION -n px4 "bash $HERE/px4_swarm.sh $PX4_ROOT $N $LOG 2>&1 | tee $LOG/px4_swarm.log"
eval "$ENV"
timeout $TRIGGER_TIMEOUT_S ros2 run pterosim_ego start_swarm --ros-args -p drones:="$N" -p use_sim_time:=true
echo "swarm triggered; logs in $LOG"
