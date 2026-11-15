#!/bin/bash
# N PX4 SITL x500s (airframe 22100) from px4_build.sh's build, instance i in uXRCE namespace x500_<i>, connecting to
# PteroSim's HIL server on 4560 + i. Started after the launch, whose agents they connect to.
#
#   ./px4_swarm.sh <px4_root> <drones> <log_dir>
set -eu
[ $# -eq 3 ] || { sed -n 5p "$0"; exit 1; }
PX4=$(realpath "$1")/build/px4_sitl_ego N=$2 LOG=$3
[ -x "$PX4/bin/px4" ] || { echo "no $PX4/bin/px4: run px4_build.sh first"; exit 1; }
AIRFRAME=22100        # PteroSim's x500 (PteroSimAircrafts x500/firmwares/px4_x500)
XRCE_PORT=8888        # the launch's XRCE_AGENT_PORT; agent g listens on XRCE_PORT + g (rcS reads PX4_UXRCE_DDS_PORT)
DRONES_PER_AGENT=2    # the launch's DRONES_PER_AGENT
# Chosen: started at once, a uXRCE client can time out creating its trajectory_setpoint reader, which PX4 never retries.
STAGGER_S=1
# UXRCE_DDS_SYNCT 0: with time sync on, a share of the clients never finish their handshake.
# EKF2_HGT_REF 0 (baro): the GPS reference wandered -0.1..6 m around a 2 m hover with PteroSim's GPS noise.
# SENS_BAR_AUTOCAL 0: autocal moved the baro onto the GPS at spin-up, copying its metres of vertical error.
# EKF2_GPS_CTRL 5 (position + velocity, no altitude): GPS altitude still pulled the baro-referenced estimate 1.5-3 m off.
for ((i = 0; i < N; i++)); do
    root=$PX4/rootfs/$i
    # A parameter store left by an earlier run would carry its calibration and gains into this one.
    mkdir -p "$root" && rm -f "$root"/parameters*.bson
    (cd "$root" && exec env -u PX4_SIM_MODEL PX4_SIM_HOSTNAME=127.0.0.1 PX4_SYS_AUTOSTART=$AIRFRAME \
        PX4_UXRCE_DDS_NS="x500_$i" PX4_UXRCE_DDS_PORT=$((XRCE_PORT + i / DRONES_PER_AGENT)) PX4_PARAM_UXRCE_DDS_SYNCT=0 \
        PX4_PARAM_EKF2_HGT_REF=0 PX4_PARAM_SENS_BAR_AUTOCAL=0 PX4_PARAM_EKF2_GPS_CTRL=5 \
        "$PX4/bin/px4" -i "$i" -d "$PX4/etc" </dev/null >"$LOG/px4_$i.log" 2>&1) &
    sleep $STAGGER_S
done
wait
