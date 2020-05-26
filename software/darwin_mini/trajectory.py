from math import sqrt
from .ik import darwin_leg_ik


PELVIS_HEIGHT_REST = 0.118
joint_names = ['hip', 'thigh', 'knee', 'ankle', 'foot']


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
        return x_pel, z_pel

    def get_swing_foot_pos(self, t):
        L = self.step_length
        h = self.step_height
        xs = self.x_swing_start
        tf = self.duration
        x_foot = xs + (3 * L / tf**2) * t**2 - (2 * L / tf**3) * t**3
        z_foot = (4 * h * t / tf) * (1 - t / tf)
        return x_foot, z_foot

    def get_stable_foot_disp_vs_pelvis(self, pelvis_pos):
        x_pel, z_pel = pelvis_pos
        dx = self.x_stable - x_pel
        dz = PELVIS_HEIGHT_REST - z_pel
        return dx, dz

    def get_swing_foot_disp_vs_pelvis(self, swing_foot_pos, pelvis_pos):
        x_foot, z_foot = swing_foot_pos
        x_pel, z_pel = pelvis_pos
        dx = x_foot - x_pel
        dz = PELVIS_HEIGHT_REST - z_pel - z_foot
        return dx, dz

    def __call__(self, t):
        """
        Returns a map of joint positions at time t.
        """

        # Pelvis motion
        pelvis_pos = self.get_pelvis_pos(t)
        dx, dz = self.get_stable_foot_disp_vs_pelvis(pelvis_pos)
        print('stable foot vs pelvis:  dx={:.4f}, dz={:.4f}'.format(dx, dz))
        q_stable = darwin_leg_ik(dx, 0., dz)

        # Swing foot motion
        swing_foot_pos = self.get_swing_foot_pos(t)
        dx, dz = self.get_swing_foot_disp_vs_pelvis(swing_foot_pos, pelvis_pos)
        print('swing foot vs pelvis:   dx={:.4f}, dz={:.4f}, z_foot={:.4f}'.format(dx, dz, swing_foot_pos[1]))
        q_swing = darwin_leg_ik(dx, 0., dz)

        # Return a map of joint positions
        stable_side = 'l' if self.swing_side == 'r' else 'r'
        stable_joints = ['_'.join([stable_side, n, 'joint']) for n in joint_names]
        swing_joints = ['_'.join([self.swing_side, n, 'joint']) for n in joint_names]

        return {**dict(zip(stable_joints, q_stable)),
                **dict(zip(swing_joints, q_swing))}
