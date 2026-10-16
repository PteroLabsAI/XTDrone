"""PteroSim's simulation time on /clock, for every node on use_sim_time.

Read from the simulator's control API (GetSimulationClock: the physics time of the aircraft furthest along since Start),
not from a drone's camera, whose stream closes when that drone crashes. Each call is answered on the simulator's game
thread, once per frame, so /clock ticks at its frame rate.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from rosgraph_msgs.msg import Clock

from pterosim import PteroSim
from pterosim_ego import US_PER_S, stamp_from_us

CLOCK_QUEUE = 1  # rclcpp's own /clock subscription depth


def main():
    rclpy.init()
    node = rclpy.create_node("pterosim_clock")
    node.declare_parameter("control", rclpy.Parameter.Type.STRING)
    control = node.get_parameter("control").value
    sim = PteroSim(control)
    pub = node.create_publisher(Clock, "/clock", QoSProfile(depth=CLOCK_QUEUE))
    node.get_logger().info(f"publishing {control}'s simulation time on /clock")
    last_us = 0
    try:
        while rclpy.ok():
            now_us = round(sim.clock().simulation_time * US_PER_S)
            # 0 while stopped; a Start restarts it from 0, which a running ROS graph cannot follow back.
            if now_us > last_us:
                pub.publish(Clock(clock=stamp_from_us(now_us)))
                last_us = now_us
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
