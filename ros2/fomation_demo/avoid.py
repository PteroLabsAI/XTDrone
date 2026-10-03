"""ROS 2 port of coordination/formation_demo/avoid.py, velocity control: the push that keeps vehicles apart.

Pairs closer than AVOID_RADIUS get pushed sideways, perpendicular to the line between them, harder the closer
they are; follower.py adds each follower's sum to its command.
"""
import numpy

AVOID_RADIUS = 1.5  # m, the ROS 1 node's avoid_radius
AID_VEC1 = numpy.array([1.0, 0.0, 0.0])
AID_VEC2 = numpy.array([0.0, 1.0, 0.0])


def avoid_pushes(pos):
    """Every vehicle's push, from all their positions in one ENU frame."""
    push = [numpy.zeros(3) for _ in pos]
    for i in range(len(pos)):
        for j in range(i + 1, len(pos)):
            dir_vec = pos[i] - pos[j]
            dist = numpy.linalg.norm(dir_vec)
            k = 1 - dist / AVOID_RADIUS
            if k > 0:
                # Sideways about whichever aid axis is further from the line between them, as the ROS 1 node did.
                cos1 = abs(dir_vec.dot(AID_VEC1)) / dist
                cos2 = abs(dir_vec.dot(AID_VEC2)) / dist
                side = numpy.cross(dir_vec, AID_VEC1 if cos1 < cos2 else AID_VEC2)
                push[i] += k * side / numpy.linalg.norm(side)
                push[j] -= k * side / numpy.linalg.norm(side)
    return push
