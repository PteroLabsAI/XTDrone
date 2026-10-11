# The ROS 2 side of XTDrone for PteroSim: Humble with every package under ros2/ built.
#   git submodule update --init ros2/ego-planner-swarm
#   docker build -t xtdrone-ros2 ros2
#   docker run --rm -it --network host xtdrone-ros2 ros2 launch ego_planner single_run_in_sim.launch.py
# --network host: DDS discovery and PX4's link see the same network as the rest of the machine.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY apt.txt /tmp/apt.txt
RUN apt-get update \
 && grep -v -e '^#' -e '^$' /tmp/apt.txt | xargs apt-get install -y --no-install-recommends \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . src/
# Every other dependency comes from the packages' own package.xml. Skipped: the CMake names apt.txt covers, and
# dynamic_reconfigure and tf, ROS 1 packages that local_sensing and odom_visualization still list.
RUN apt-get update \
 && rosdep update --rosdistro humble \
 && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
      --skip-keys "PCL Eigen3 OpenCV Boost Armadillo dynamic_reconfigure tf" \
 && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/humble/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
