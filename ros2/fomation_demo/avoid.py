"""ROS 2 port of coordination/formation_demo/avoid.py, velocity control: the push that keeps vehicles apart.

Pairs closer than the avoid radius get pushed sideways, perpendicular to the line between them, harder the closer
they are; follower.py adds each follower's sum to its command.
"""
import itertools

import numpy

# m; the ROS 1 node's avoid_radius was 1.5, which let followers crossing at 2 m/s close to 0.44 m before the push began
# (18 x500 in PteroSim, 2026-10-03). Never wider than the closest two slots of the fleet's patterns, so a formed figure
# feels no push: 2.5 m for 18 vehicles (slots >= 3 m apart), 2.0 m for 6 (the "T" has slots 2 m apart).
AVOID_RADIUS_MAX = 2.5
AID_VEC1 = numpy.array([1.0, 0.0, 0.0])
AID_VEC2 = numpy.array([0.0, 1.0, 0.0])


def avoid_radius(patterns):
    """The push's reach for a fleet flying these patterns (3 x followers arrays, the leader at the origin)."""
    closest = min(numpy.linalg.norm(a - b) for p in patterns
                  for a, b in itertools.combinations([numpy.zeros(3)] + list(p.T), 2))
    return min(AVOID_RADIUS_MAX, closest)


def avoid_pushes(pos, radius):
    """Every vehicle's push, from all their positions in one ENU frame."""
    push = [numpy.zeros(3) for _ in pos]
    for i in range(len(pos)):
        for j in range(i + 1, len(pos)):
            dir_vec = pos[i] - pos[j]
            dist = numpy.linalg.norm(dir_vec)
            k = 1 - dist / radius
            if k > 0:
                # Sideways about whichever aid axis is further from the line between them, as the ROS 1 node did.
                cos1 = abs(dir_vec.dot(AID_VEC1)) / dist
                cos2 = abs(dir_vec.dot(AID_VEC2)) / dist
                side = numpy.cross(dir_vec, AID_VEC1 if cos1 < cos2 else AID_VEC2)
                push[i] += k * side / numpy.linalg.norm(side)
                push[j] -= k * side / numpy.linalg.norm(side)
    return push
