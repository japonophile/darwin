from math import sin, acos, atan


PELVIS_HEIGHT_REST = 0.118
FOOT_SPREAD = 0.32  # lateral distance of each foot wrt pelvis

EPS = 1e-6
a1 = 0.006
a2 = 0.045
a3 = 0.042
a4 = 0.006
JOINT_NAMES = ['hip', 'thigh', 'knee', 'ankle', 'foot']


def darwin_leg_ik(dx, dy, dz, side):
    assert side in ['l', 'r']
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
    if side == 'r':
        q[1] = -q[1]
        q[2] = -q[2]
        q[3] = -q[3]
    # angle of the foot
    q[4] = -q[0] if abs(q[0]) > EPS else 0.0
    return q


def get_foot_disp_vs_pelvis(foot_pos, pelvis_pos):
    x_foot, _, z_foot = foot_pos
    x_pel, _, z_pel = pelvis_pos
    dx = x_pel - x_foot
    dy = 0.0
    dz = PELVIS_HEIGHT_REST - z_pel - z_foot
    return dx, dy, dz


def darwin_ik(pos):
    """
    Returns a map of joint angles for position.
    """
    pelvis_pos = pos['pelvis']
    q = {}
    for side in ['l', 'r']:
        foot_pos = pos[side + '_foot']
        dx, dy, dz = get_foot_disp_vs_pelvis(foot_pos, pelvis_pos)
        joint_angles = darwin_leg_ik(dx, dy, dz, side)
        joints = ['_'.join([side, n, 'joint']) for n in JOINT_NAMES]
        q = { **q, **dict(zip(joints, joint_angles)) }
    return q
