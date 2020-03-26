# Modified example source code from the book
# "Real-World Instrumentation with Python"
# by J. M. Hughes, published by O'Reilly Media, December 2010,
# ISBN 978-0-596-80956-0.

import rospy


class PID(object):
    """
    PID control class

    This class implements a simplistic PID control algorithm. When first
    instantiated all the gain variables are set to zero, so calling
    the method compute_output will just return zero.
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        # initialize gains
        self._kp = kp
        self._ki = ki
        self._kd = kd

        # initialize error, results, and time descriptors
        self._prev_err = 0.0
        self._cp = 0.0
        self._ci = 0.0
        self._cd = 0.0
        self._cur_time = 0.0
        self._prev_time = 0.0

        self.initialize()

    def initialize(self):
        """
        Initialize pid controller.
        """
        # reset delta t variables
        self._cur_time = rospy.get_time()
        self._prev_time = self._cur_time

        self._prev_err = 0.0

        # reset result variables
        self._cp = 0.0
        self._ci = 0.0
        self._cd = 0.0

    def set_kp(self, invar):
        """
        Set proportional gain.
        """
        self._kp = invar

    def set_ki(self, invar):
        """
        Set integral gain.
        """
        self._ki = invar

    def set_kd(self, invar):
        """
        Set derivative gain.
        """
        self._kd = invar

    def compute_output(self, error):
        """
        Performs a PID computation and returns a control value based on
        the elapsed time (dt) and the error signal from a summing junction
        (the error parameter).
        """
        self._cur_time = rospy.get_time()  # get t
        dt = self._cur_time - self._prev_time  # get delta t
        de = error - self._prev_err  # get delta error

        self._cp = error  # proportional term
        self._ci += error * dt  # integral term

        self._cd = 0
        if dt > 0:  # no div by zero
            self._cd = de / dt  # derivative term

        self._prev_time = self._cur_time  # save t for next pass
        self._prev_err = error  # save t-1 error

        # sum the terms and return the result
        return ((self._kp * self._cp) + (self._ki * self._ci) +
                (self._kd * self._cd))
