"""ROS 2 port of control/keyboard/multirotor_keyboard_control.py, velocity control.

    python3 multirotor_keyboard_control.py <multirotor_type> <multirotor_num>

The formation digits come from formation_dict: 0 is "origin", the flat grid the fleet took off from,
which is the one to land from; the other patterns stack vehicles above each other.
"""
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "fomation_demo"))
import formation_dict  # noqa: E402

MAX_LINEAR = 20.0  # m/s, the ROS 1 node's limits and steps; floats, or a clamped value turns int and Twist rejects it
MAX_ANG_VEL = 3.0  # rad/s
LINEAR_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01
KEY_POLL_S = 0.1  # the ROS 1 node's select() timeout, which is also the publish period
CTRL_C = '\x03'

HELP = """
Control Your XTDrone!  To %s  (press g to control %s)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward
a/d : increase/decrease leftward
i/, : increase/decrease upward
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover and remove the mask of keyboard control
%s
CTRL-C to quit
"""
COMMANDS = {'r': 'AUTO.RTL', 't': 'ARM', 'y': 'DISARM', 'v': 'AUTO.TAKEOFF', 'b': 'OFFBOARD', 'n': 'AUTO.LAND'}
STEPS = {'w': (0, LINEAR_STEP_SIZE), 'x': (0, -LINEAR_STEP_SIZE), 'a': (1, LINEAR_STEP_SIZE), 'd': (1, -LINEAR_STEP_SIZE),
         'i': (2, LINEAR_STEP_SIZE), ',': (2, -LINEAR_STEP_SIZE), 'j': (3, ANG_VEL_STEP_SIZE), 'l': (3, -ANG_VEL_STEP_SIZE)}
LIMITS = (MAX_LINEAR, MAX_LINEAR, MAX_LINEAR, MAX_ANG_VEL)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], KEY_POLL_S)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    multirotor_type, multirotor_num = sys.argv[1], int(sys.argv[2])
    formations = formation_dict.FORMATIONS.get(multirotor_num, {})
    digits = {str(i): name for i, name in enumerate(["origin"] + [f for f in formations if f != "origin"])} if formations else {}
    digit_help = "  ".join("%s: %s" % kv for kv in digits.items()) or "no formation for %d vehicles" % multirotor_num

    rclpy.init()
    node = rclpy.create_node(multirotor_type + '_multirotor_keyboard_control')
    names = [multirotor_type + '_' + str(i) for i in range(multirotor_num)]
    vel_pubs = [node.create_publisher(Twist, '/xtdrone/' + n + '/cmd_vel_flu', 1) for n in names]
    cmd_pubs = [node.create_publisher(String, '/xtdrone/' + n + '/cmd', 3) for n in names]
    leader_vel_pub = node.create_publisher(Twist, '/xtdrone/leader/cmd_vel_flu', 1)
    leader_cmd_pub = node.create_publisher(String, '/xtdrone/leader/cmd', 1)

    settings = termios.tcgetattr(sys.stdin)
    vel = [0.0, 0.0, 0.0, 0.0]  # forward, leftward, upward, yaw rate
    ctrl_leader = False
    cmd_vel_mask = False

    def show(note=''):
        who = ("the leader", "all drones") if ctrl_leader else ("all drones", "the leader")
        print(HELP % (*who, digit_help))
        print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f  %s" % (*vel, note))

    show()
    try:
        while True:
            key = get_key(settings)
            if key == CTRL_C:
                break
            cmd = ''
            if key in digits:
                # Only the leader knows the formations; a vehicle would take the name for a flight mode.
                leader_cmd_pub.publish(String(data=digits[key]))
                # A formation hands the followers to the leader, so the keyboard stops steering them.
                cmd_vel_mask = True
                show(digits[key])
            elif key == 'g':
                ctrl_leader = not ctrl_leader
                show()
            elif key in STEPS:
                axis, step = STEPS[key]
                vel[axis] = max(-LIMITS[axis], min(LIMITS[axis], vel[axis] + step))
                show()
            elif key in ('k', 's'):
                cmd_vel_mask = False
                vel = [0.0, 0.0, 0.0, 0.0]
                cmd = 'HOVER'
                show(cmd)
            elif key in COMMANDS:
                cmd = COMMANDS[key]
                show(cmd)

            # A velocity is streamed while it is not zero: the communication nodes hover once it stops coming.
            # Zero goes once, with the key that made it; streaming it, as the ROS 1 node did, fought every follower.
            if any(vel) or key in STEPS or cmd == 'HOVER':
                twist = Twist()
                twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = vel
                if ctrl_leader:
                    leader_vel_pub.publish(twist)
                elif not cmd_vel_mask:
                    for vel_pub in vel_pubs:
                        vel_pub.publish(twist)
            if cmd:
                for cmd_pub in [leader_cmd_pub] if ctrl_leader else cmd_pubs:
                    cmd_pub.publish(String(data=cmd))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
