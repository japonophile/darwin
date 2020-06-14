from numpy import array, zeros


class Point(object):

    """
    Point

    pos : array, shape=(3,), optional
        Initial position in the world frame.
    vel : array, shape=(3,), optional
        Initial velocity in the world frame.
    accel : array, shape=(3,), optional
        Initial acceleration in the world frame.
    """

    def __init__(self, pos=None, vel=None, accel=None):
        self.__pos = zeros(3) if pos is None else array(pos)
        self.__pd = zeros(3) if vel is None else array(vel)
        self.__pdd = zeros(3) if accel is None else array(accel)

    def copy(self):
        """
        Copy constructor.

        color : char, optional
            Color of the copy, in ['r', 'g', 'b'].
        visible : bool, optional
            Should the copy be visible?
        """
        return Point(self.p, self.pd)

    @property
    def p(self):
        """Position coordinates `[x y z]` in the world frame."""
        return self.__pos.copy()

    @property
    def pos(self):
        """Position coordinates `[x y z]` in the world frame."""
        return self.p

    def set_pos(self, pos):
        """
        Set the position of the body in the world frame.

        pos : array, shape=(3,)
            3D vector of position coordinates.
        """
        self.__pos = array(pos)

    @property
    def pd(self):
        """Point velocity."""
        return self.__pd.copy()

    def set_vel(self, pd):
        """
        Update the point velocity.

        pd : array, shape=(3,)
            Velocity coordinates in the world frame.
        """
        self.__pd = array(pd)

    @property
    def pdd(self):
        """Point acceleration."""
        return self.__pdd.copy()

    def set_accel(self, pdd):
        """
        Update the point acceleration.

        pdd : array, shape=(3,)
            Acceleration coordinates in the world frame.
        """
        self.__pdd = array(pdd)

    def integrate_constant_accel(self, pdd, dt):
        """
        Apply Euler integration for a constant acceleration.

        pdd : array, shape=(3,)
            Point acceleration in the world frame.
        dt : scalar
            Duration in [s].
        """
        self.set_pos(self.p + (self.pd + .5 * pdd * dt) * dt)
        self.set_vel(self.pd + pdd * dt)
        self.set_accel(pdd)

    def integrate_constant_jerk(self, pddd, dt):
        """
        Apply Euler integration for a constant jerk.

        pddd : array, shape=(3,)
            Point jerk in the world frame.
        dt : scalar
            Duration in [s].
        """
        self.set_pos(self.p + dt * (
            self.pd + .5 * dt * (self.pdd + dt * pddd / 3.)))
        self.set_vel(self.pd + dt * (self.pdd + dt * pddd / 2.))
        self.set_accel(self.pdd + dt * pddd)
