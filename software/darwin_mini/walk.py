from pypot.primitive import LoopPrimitive
from .trajectory import ConstantTrajectory, FootstepTrajectory
from .ik import darwin_ik
from .ik import PELVIS_HEIGHT_REST, FOOT_SPREAD
from numpy import rad2deg


class WalkingState(object):
    SINGLE_SUPPORT = 1
    DOUBLE_SUPPORT = 2


class WalkStraight(LoopPrimitive):
    def __init__(self, robot, distance, step_duration, frequency=50):
        LoopPrimitive.__init__(self, robot, frequency)

        self.step_length = 0.1
        self.step_height = 0.01
        self.pelvis_height = 0.098
        self.motors = dict([(m.name, self.get_mockup_motor(m)) for m in self.robot.motors])
        self.distance_left = distance
        self.step_duration = step_duration
        ssp_ratio = .75
        self.ssp_duration = ssp_ratio * step_duration
        self.dsp_duration = (1 - ssp_ratio) * step_duration
        self.dt = 1. / frequency
        self.pos = None

    @property
    def stable_side(self):
        return 'l' if self.swing_side == 'r' else 'r'

    def setup(self):
        self.swing_side = 'r'
        self.x_stable = 0.
        self.x_swing_start = 0.
        self.pos = {
            'pelvis': [0., 0., PELVIS_HEIGHT_REST],
            'l_foot': [0., FOOT_SPREAD, 0.],
            'r_foot': [0., -FOOT_SPREAD, 0.],
        }
        self.start_step(0)

    def update(self):
        t = self.elapsed_time
        # print(t)
        if self.state == WalkingState.DOUBLE_SUPPORT:
            self.run_double_support(t)
        elif self.state == WalkingState.SINGLE_SUPPORT:
            self.run_single_support(t)
        else:
            raise Exception("Unknown state: {}".format(self.state))

    def update_position(self, t):
        self.pos = self.current_trajectory(t)
        q = darwin_ik(self.pos)
        self.goto_position(q)

    def run_double_support(self, t):
        t -= self.current_step * self.step_duration
        if t > self.dsp_duration:
            self.start_single_support()
        self.update_position(t)

    def run_single_support(self, t):
        step = t // self.step_duration
        t = t % self.step_duration
        done = False
        if step > self.current_step:
            done = self.check_walking_done()
            if not done:
                self.start_next_step(step)
        if not done:
            self.update_position(t)

    def get_next_step_length(self):
        next_step_max_length = self.step_length if self.current_step > 0 else 0.5 * self.step_length
        if 2 * self.distance_left > next_step_max_length:
            next_step_length = next_step_max_length
        else:
            next_step_length = 2 * self.distance_left
        return next_step_length

    def check_walking_done(self):
        # the pelvis only moves half of the footstep length
        self.distance_left = max(0, self.distance_left - 0.5 * self.current_step_length)
        if self.distance_left < 1e-6:
            # We have arrived.
            print('Done!')
            self.stop()
            return True
        return False

    def start_step(self, step):
        print('\n*** Start step {} ***'.format(step + 1))
        self.current_step = step
        self.current_step_length = self.get_next_step_length()
        self.start_double_support()

    def start_next_step(self, step):
        self.swing_side = 'l' if self.swing_side == 'r' else 'r'
        next_swing_start = self.x_stable
        self.x_stable = self.x_swing_start + self.current_step_length
        self.x_swing_start = next_swing_start
        self.start_step(step)

    def start_single_support(self):
        print(" >> start single support phase <<")
        self.current_trajectory = FootstepTrajectory(
            self.current_step_length, self.step_height, self.x_stable,
            self.x_swing_start, self.pelvis_height, self.swing_side, self.ssp_duration)
        self.state = WalkingState.SINGLE_SUPPORT

    def get_swing_target(self):
        traj = FootstepTrajectory(
            self.current_step_length, self.step_height, self.x_stable,
            self.x_swing_start, self.pelvis_height, self.swing_side, self.ssp_duration)
        return traj(self.ssp_duration)[self.swing_side + '_foot']

    def start_double_support(self):
        print(" >> start double support phase <<")
        self.current_trajectory = ConstantTrajectory(self.pos)
        self.swing_target = self.get_swing_target()
        self.state = WalkingState.DOUBLE_SUPPORT

    def goto_position(self, qs):
        for name, q in qs.items():
            m = self.motors[name]
            m.goal_position = rad2deg(q)
