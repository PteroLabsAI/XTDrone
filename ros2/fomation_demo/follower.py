"""Every follower of the formation in one node: each flies to its slot of /xtdrone/formation_pattern around the
leader, pushed clear of the others, on /xtdrone/<uav>/cmd_vel_enu.

    python3 follower.py <uav_type> <uav_num>
"""
import sys

import numpy
import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from avoid import avoid_pushes
from geo import enu_between
from px4 import QOS, px4_topic

KP = 1.0  # the ROS 1 follower's Kp
KP_AVOID = 2.0  # the ROS 1 follower's Kp_avoid
# m/s, XTDrone's vel_max, now on the move relative to the leader only: slot changes keep its pace. Measured in
# PteroSim: closest approach 1.2 m at 1 m/s, 0.64 m when the whole command was allowed 5 m/s.
CORRECTION_MAX = 1.0
# The communication node streams setpoints at 30 Hz; PX4 sends positions at 50 Hz (dds_topics.yaml rate_limit).
CONTROL_HZ = 30.0


class Followers(Node):
    def __init__(self, uav_type, uav_num):
        super().__init__("followers")
        self.uav_num = uav_num
        self.fix = [None] * uav_num
        self.leader_vel = None
        self.formation_pattern = None
        self.cmd_vel_enu = Twist()
        # Each PX4 puts its local origin at its own first GPS fix, so local positions of two vehicles
        # cannot be subtracted; their global fixes can.
        for i in range(uav_num):
            name = uav_type + '_' + str(i)
            self.create_subscription(VehicleGlobalPosition, px4_topic(name, "out", "vehicle_global_position", VehicleGlobalPosition),
                                     lambda m, i=i: self.fix.__setitem__(i, m), QOS)
        # Turned into ENU, the axes of every follower's command; a velocity is the same in every local frame.
        self.create_subscription(VehicleLocalPosition, px4_topic(uav_type + "_0", "out", "vehicle_local_position", VehicleLocalPosition),
                                 lambda m: setattr(self, "leader_vel", (m.vy, m.vx, -m.vz)), QOS)
        self.create_subscription(Float32MultiArray, "/xtdrone/formation_pattern", self.formation_pattern_callback, 10)
        self.vel_enu_pubs = [self.create_publisher(Twist, '/xtdrone/' + uav_type + '_' + str(i) + '/cmd_vel_enu', 10)
                             for i in range(1, uav_num)]
        self.create_timer(1.0 / CONTROL_HZ, self.timer_callback)

    def formation_pattern_callback(self, msg):
        # dtype=float is required. rclpy hands float32[] over as a numpy array of dtype
        # float32, and numpy.float32 is NOT a subclass of Python float (numpy.float64 is).
        # Without the cast, the arithmetic below yields numpy.float32, and publishing the
        # Twist aborts the process in geometry_msgs__msg__vector3__convert_from_py on
        # PyFloat_Check(field).
        self.formation_pattern = numpy.array(msg.data, dtype=float).reshape(3, self.uav_num - 1)

    def timer_callback(self):
        if self.formation_pattern is None or self.leader_vel is None or any(f is None for f in self.fix):
            return
        # Avoid runs here, in the followers' tick, on the same fixes; it was a node that published every push.
        push = avoid_pushes([numpy.array(enu_between(self.fix[0], f)) for f in self.fix])
        v = self.leader_vel
        for i, pub in enumerate(self.vel_enu_pubs, start=1):
            to_slot = numpy.array(enu_between(self.fix[i], self.fix[0])) + self.formation_pattern[:, i - 1]
            correction = KP * to_slot + KP_AVOID * push[i]
            speed = numpy.linalg.norm(correction)
            if speed > CORRECTION_MAX:
                correction *= CORRECTION_MAX / speed
            # The leader's velocity goes in whole; the position term alone trails a moving slot by speed / Kp.
            self.cmd_vel_enu.linear.x = v[0] + float(correction[0])
            self.cmd_vel_enu.linear.y = v[1] + float(correction[1])
            self.cmd_vel_enu.linear.z = v[2] + float(correction[2])
            pub.publish(self.cmd_vel_enu)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    rclpy.init()
    rclpy.spin(Followers(sys.argv[1], int(sys.argv[2])))


if __name__ == '__main__':
    main()
