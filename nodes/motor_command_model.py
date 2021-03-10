"""Functions for modeling ROSBot"""

import numpy as np
from math import cos, sin

def model_parameters():
    """ Returns two constant model parameters """
    k = 1.0
    d = 0.5
    return k, d

def system_matrix(theta):
    """ Return numpy array with A(theta) matrix for a differential drive robot """
    return A

def system_field(z, u):
    """ Compute the field at a given state for the dynamical model """
    return dot_z

def euler_step(z, u, stepSize):
    """ Integrates dynamical modle for one time step using Euler's method """
    return zp

def twist_to_speeds(speed_linear, speed_angular):
    """ Returns normalized speeds for left and right motors
        Speed needs to be between -1.0 and 1.0 """

    # Error Checking
    if (speed_linear < -1.0 or speed_linear > 1.0):
        return False # print Error
    if (speed_angular < -1.0 or speed_angular > 1.0):
        return False # print Error

    # Normalizing
    if (speed_angular < 0):
        left = (speed_linear + speed_angular) / 2.0
        right = (speed_linear) / 2.0
    else: # if angular speed is negative, make left greater than right
        right = (speed_linear + speed_angular) / 2.0
        left = (speed_linear) / 2.0

    return left, right
