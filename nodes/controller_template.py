#!/usr/bin/env python


class PIDController(object):
    """ Computes proportional, derivative, and integral terms for a PID controller """

    def __init__(self, kp=1, kd=1, ki=1):
        """Initializes gains and internal state (previous error and integral error)"""
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.error_signal_previous = None
        self.error_signal_integral = 0

    def proportional(self, error_signal):
        """ Compute proportional term (with gain) """
        #TODO: This is a stub. Write your code here.
        return 0

    def derivative(self, error_signal, time_delay):
        """ Compute derivative term (with gain) """
        #TODO: This is a stub. Write your code here.
        return 0

    def integral(self, error_signal, time_delay):
        """ Compute integral term (with gain) """
        #TODO: This is a stub. Write your code here.
        return 0
