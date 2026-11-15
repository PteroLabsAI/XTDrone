#!/bin/bash
# PX4 SITL for the swarm in its own build dir, build/px4_sitl_ego, with dds_topics.yaml beside this script as the
# uXRCE-DDS topic list: swapped in that dir's build.ninja only, so the source tree and build/px4_sitl_default stay as they are.
#
#   ./px4_build.sh <px4_root>
set -eu
[ $# -eq 1 ] || { sed -n 5p "$0"; exit 1; }
PX4_ROOT=$(realpath "$1") HERE=$(cd "$(dirname "$0")" && pwd)
BUILD=$PX4_ROOT/build/px4_sitl_ego
DEFAULT_TOPICS=$PX4_ROOT/src/modules/uxrce_dds_client/dds_topics.yaml
cmake -S "$PX4_ROOT" -B "$BUILD" -G Ninja -DCONFIG=px4_sitl_default -DCMAKE_BUILD_TYPE=RelWithDebInfo
if ! grep -q "$HERE/dds_topics.yaml" "$BUILD/build.ninja"; then
    grep -q "$DEFAULT_TOPICS" "$BUILD/build.ninja" || { echo "$BUILD/build.ninja names no $DEFAULT_TOPICS"; exit 1; }
    sed -i "s|$DEFAULT_TOPICS|$HERE/dds_topics.yaml|g" "$BUILD/build.ninja"
fi
ninja -C "$BUILD"
# A CMake rerun inside ninja writes build.ninja afresh, with PX4's own list.
grep -q "$HERE/dds_topics.yaml" "$BUILD/build.ninja" || { echo "CMake regenerated $BUILD/build.ninja: run this again"; exit 1; }
