#!/usr/bin/env python
""" A first template for a PID controller """

class PID(object):
    """ Computes proportional, derivative and integral terms for PID controller """

    def __init__(self, kp=1.0, kd=1.0, ki=1.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.error_signal_previous = None
        self.error_signal_integral = 0

    def proportional(self, error_signal):
        f_b = -self.kp * error_signal
        return f_b

    def integral(self, error_signal, time_delay):
        self.error_signal_integral += (error_signal * time_delay)
        f_i = -self.ki * self.error_signal_integral
        return f_i

    def derivative(self, error_signal, time_delay):
        # TODO
        return f_d
