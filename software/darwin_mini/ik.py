#!/usr/bin/env python
# coding: utf-8

from math import sin, acos, atan


EPS = 1e-6
a1 = 0.006
a2 = 0.045
a3 = 0.042
a4 = 0.006


def darwin_leg_ik(dx, dy, dz):
    assert dz >= 0  # dx and dy are allowed to be negative
    q = [.0] * 5
    q[0] = atan(dy / (a1 + a2 + a3 - a4 - dz))
    # length of the leg as seen from the front view
    L = dy / sin(q[0]) if abs(dy) > EPS else a1 + a2 + a3 - a4 - dz
    assert L <= a1 + a2 + a3 - a4, "Leg cannot be extended beyond limit. Use larger dz and/or smaller dy."
    # angle of the leg in the leg plane
    alpha = atan(dx / (L - a1 + a4))
    # distance from thigh to ankle (just 2 links, thigh and lower leg)
    d = dx / sin(alpha) if abs(dx) > EPS else L - a1 + a4
    assert d <= a2 + a3, "Leg cannot be extended beyond limit. Use larger dz, smaller dy and/or smaller dx."
    # angles of the thigh-knee-ankle triangle
    beta1 = acos((a2**2 + d**2 - a3**2) / (2 * a2 * d))
    beta3 = acos((a3**2 + d**2 - a2**2) / (2 * a3 * d))
    # angles of the thigh, knee and ankle joints
    q[1] = beta1 + alpha
    q[3] = beta3 - alpha
    q[2] = -(q[1] + q[3])
    # angle of the foot
    q[4] = -q[0] if abs(q[0]) > EPS else 0.0
    return q
