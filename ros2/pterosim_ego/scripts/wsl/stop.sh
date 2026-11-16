#!/bin/bash
# Stop every process of the swarm: its PX4s, the tmux session, and the launch and its nodes, which outlive the session.
NODES="px4_sitl_ego/bin/px4|ros2 launch pterosim_ego|ego_planner_node|traj_server|px4_bridge|depth_bridge|pterosim_clock|MicroXRCEAgent|start_swarm"
STOP_POLLS=20 POLL_S=0.5  # chosen: 10 s for the nodes to end on SIGTERM before SIGKILL
tmux kill-session -t ego 2>/dev/null
pkill -f "$NODES"
for ((i = 0; i < STOP_POLLS; i++)); do pgrep -f "$NODES" >/dev/null || break; sleep $POLL_S; done
pkill -9 -f "$NODES"
echo "left: $(pgrep -fc "$NODES")"
