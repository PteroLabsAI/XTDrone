"""Send the swarm through the forest: one /traj_start_trigger once every drone's px4_bridge reports its takeoff done.

Each planner's goal is its waypoint parameter (fsm/flight_type 2, set by the launch from goals:=). With
fsm/realworld_experiment true, ego-planner waits for this trigger before it plans (ego_replan_fsm.cpp:35, :145), so
nothing is planned from the ground. The trigger topic is one for the whole swarm (ego_replan_fsm.cpp:128).
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Empty

from pterosim_ego import drone_topic

TRIGGER_TOPIC = "/traj_start_trigger"
TRIGGER_QUEUE = 1  # ego-planner's own depth (ego_replan_fsm.cpp:129)
POLL_S = 0.5  # chosen: how often to look for takeoffs and planners; not a simulation time
TRIGGER_ACK_TIMEOUT_S = 5.0  # chosen: a local reliable delivery takes milliseconds
# Chosen: the swarm's takeoffs spread over ~10 s; one still down this long after the first is not coming.
LATE_S = 30.0


def main():
    rclpy.init()
    node = rclpy.create_node("start_swarm")
    node.declare_parameter("drones", rclpy.Parameter.Type.INTEGER)
    drones = node.get_parameter("drones").value
    flown = set()
    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    for i in range(drones):
        node.create_subscription(Empty, drone_topic(i, "takeoff_done"), lambda _, i=i: flown.add(i), latched)
    trigger = node.create_publisher(PoseStamped, TRIGGER_TOPIC, TRIGGER_QUEUE)
    node.get_logger().info(f"waiting for {drones} takeoffs and {drones} planners on {TRIGGER_TOPIC}")
    first = None  # the node clock (sim time under use_sim_time) at the first takeoff
    try:
        while rclpy.ok() and (len(flown) < drones or trigger.get_subscription_count() < drones):
            rclpy.spin_once(node, timeout_sec=POLL_S)
            first = first or (node.get_clock().now() if flown else None)
            late = first and node.get_clock().now() - first > Duration(seconds=LATE_S)
            if late and trigger.get_subscription_count() >= len(flown):
                node.get_logger().error(f"drones {sorted(set(range(drones)) - flown)} still down {LATE_S:.0f} s after the "
                                        f"first takeoff; sending the {len(flown)} that are up")
                break
        trigger.publish(PoseStamped())
        if not trigger.wait_for_all_acked(Duration(seconds=TRIGGER_ACK_TIMEOUT_S)):
            raise RuntimeError(f"{TRIGGER_TOPIC} not acknowledged by every planner within {TRIGGER_ACK_TIMEOUT_S} s")
        node.get_logger().info(f"{len(flown)} drones up, {trigger.get_subscription_count()} planners triggered")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
