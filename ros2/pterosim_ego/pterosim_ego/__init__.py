"""ROS 2 bridge between PteroSim x500s under PX4 SITL and ZJU ego-planner-swarm."""
from builtin_interfaces.msg import Time

US_PER_S = 1_000_000
NS_PER_US = 1_000


def drone_topic(drone_id, name):
    """ego-planner-swarm's per-drone topic name, /drone_<id>_<name> (plan_manage/launch/advanced_param.launch.py:96)."""
    return f"/drone_{drone_id}_{name}"


def stamp_from_us(us):
    """A ROS stamp from microseconds of simulation time: PX4's lockstep clock and PteroSim's sim_us are both."""
    return Time(sec=us // US_PER_S, nanosec=(us % US_PER_S) * NS_PER_US)
