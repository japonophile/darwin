from math import sqrt
from .ik import PELVIS_HEIGHT_REST, FOOT_SPREAD


def log_positions(pos, t):
    print("Positions at time {:.3f}:".format(t))
    for l in ['pelvis', 'l_foot', 'r_foot']:
        print("  {}: ({:.3f}, {:.3f}, {:.3f})".format(l, *pos[l]))


class ConstantTrajectory(object):
    def __init__(self, pos):
        self.pos = pos

    def get_pos(self, t):
        log_positions(self.pos, t)
        return self.pos

    def __call__(self, t):
        return self.get_pos(t)


class FootstepTrajectory(object):
    def __init__(self, step_length, step_height, x_stable, x_swing_start, pelvis_height, swing_side, duration,
                 pelvis_start_vel=[0., 0.], pelvis_end_vel=[0., 0.]):
        assert swing_side in ['l', 'r'], "swing_side must be either 'l' or 'r'"
        assert pelvis_height < PELVIS_HEIGHT_REST, "pelvis must be lower than {}".format(PELVIS_HEIGHT_REST)
        self.step_length = step_length
        self.step_height = step_height
        self.x_stable = x_stable
        self.x_swing_start = x_swing_start
        self.pelvis_height = pelvis_height
        self.swing_side = swing_side
        self.duration = duration
        self.pelvis_start_vel = pelvis_start_vel
        self.pelvis_end_vel = pelvis_end_vel
        # assume (for now):
        # - both feet are on the ground before starting the trajectory
        # - pelvis is (and stays) at y = 0
        # - pelvis start and end velocities are given as [vx, vz]

    @property
    def stable_side(self):
        return 'l' if self.swing_side == 'r' else 'r'

    @staticmethod
    def get_y_foot(side):
        return (1.0 if side == 'l' else -1.0) * FOOT_SPREAD

    def get_pelvis_pos(self, t):
        L = self.step_length
        xs = 0.5 * (self.x_stable + self.x_swing_start)
        vs = self.pelvis_start_vel[0]
        ve = self.pelvis_end_vel[0]
        tf = self.duration
        r4 = -2 * (L / (2 * tf**3) - (vs + ve) / (2 * tf**2))
        x_pel = (xs + vs * t + ((ve - vs) / (2 * tf) - r4 * 1.5 * tf) * t**2
                 - 2 * (L / (2 * tf**3) - (vs + ve) / (2 * tf**2)) * t**3)
        h = self.pelvis_height
        z_pel = sqrt(h**2 - (x_pel - self.x_stable)**2)
        y_pel = 0.0
        return x_pel, y_pel, z_pel

    def get_swing_foot_pos(self, t):
        L = self.step_length
        h = self.step_height
        xs = self.x_swing_start
        tf = self.duration
        x_foot = xs + (3 * L / tf**2) * t**2 - (2 * L / tf**3) * t**3
        z_foot = (4 * h * t / tf) * (1 - t / tf)
        y_foot = self.get_y_foot(self.swing_side)
        return x_foot, y_foot, z_foot

    def get_pos(self, t):
        """
        Returns a map of pelvis and feet cartesian positions at time t.
        """
        pelvis_pos = self.get_pelvis_pos(t)
        stable_foot_pos = (self.x_stable, self.get_y_foot(self.stable_side), 0.0)
        swing_foot_pos = self.get_swing_foot_pos(t)
        pos = {
            'pelvis': pelvis_pos,
            self.stable_side + '_foot': stable_foot_pos,
            self.swing_side + '_foot': swing_foot_pos
        }
        log_positions(pos, t)
        return pos

    def __call__(self, t):
        """
        Returns a map of cartesian positions at time t.
        """
        return self.get_pos(t)
