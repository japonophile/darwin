from numpy import sum
from functools import partial

from pypot.creatures import AbstractPoppyCreature

# from .primitives.postures import SafePowerUp


class DarwinMini(AbstractPoppyCreature):

    @classmethod
    def setup(cls, robot):
        robot._primitive_manager._filter = partial(sum, axis=0)

        # robot.attach_primitive(SafePowerUp(robot), 'safe_power_up')

        for m in robot.motors:
            m.pid = (4, 2, 0)
            m.torque_limit = 70.
            m.led = 'off'
