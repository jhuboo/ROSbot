"""This is a library of helpful classes and functions for the ME416 Lab. If the module is on non-RasberryPi systems (more
exactly, where the RPi module is not available), the motor commands are logged to the console"""

from __future__ import print_function
from threading import Thread
import time

# This module might be used not on a RPi
try:
    import RPi.GPIO as GPIO
except ImportError:
    IS_RPI = False
else:
    IS_RPI = True

import atexit

# Set the pin numbering scheme to board numbers (as opposed to Broadcom number, see the pinout)
if IS_RPI:
    GPIO.setmode(GPIO.BOARD)
    atexit.register(GPIO.cleanup)

# Assign variable names to the pins so you don't have to control them by number
R_forward_pin = 31
R_backward_pin = 29
L_forward_pin = 16
L_backward_pin = 18
# Define the GPIO pins for reading quadrature encoder
R_encoder_A = 19
R_encoder_B = 21
L_encoder_A = 3
L_encoder_B = 5


# Motor control class
class MotorSpeed:
    """A class to control motors using PWM on all channels of an H-bridge thorugh GPIO"""
    def __init__(self,
                 fw_pin,
                 bw_pin,
                 speed_factor=1.0,
                 max_duty_cycle=90,
                 motor_name="Motor"):
        #Save parameter privately
        self.speed_factor = speed_factor
        self.max_duty_cycle = 90
        self.motor_name = motor_name

        #Init pins and PWM objects
        if IS_RPI:
            GPIO.setup(fw_pin, GPIO.OUT)
            GPIO.setup(bw_pin, GPIO.OUT)
            self.fw_pwm = GPIO.PWM(fw_pin, 100)
            self.bw_pwm = GPIO.PWM(bw_pin, 100)
            self.fw_pwm.start(0)
            self.bw_pwm.start(0)
        else:
            print('Motor "%s" initialized' % motor_name)

    def set_speed(self, speed):
        """ Set speed. speed=-1 is max_duty_cycle backward, speed=1 is max_duty_cycle foward, speed=0 is stop """
        duty_cycle = min(
            int(abs(speed) * self.speed_factor * self.max_duty_cycle),
            self.max_duty_cycle)
        if speed < 0:
            duty_cycle_bw = duty_cycle
            duty_cycle_fw = 0
        elif speed > 0:
            duty_cycle_bw = 0
            duty_cycle_fw = duty_cycle
        else:
            duty_cycle_bw = 0
            duty_cycle_fw = 0

        if IS_RPI:
            self.bw_pwm.ChangeDutyCycle(duty_cycle_bw)
            self.fw_pwm.ChangeDutyCycle(duty_cycle_fw)
        else:
            print('%s duty cycles: Forward = %d, Backward = %d.' %
                  (self.motor_name, duty_cycle_fw, duty_cycle_bw))

#Specialized motor classes
class MotorSpeedLeft(MotorSpeed):
    """Inherited class specialized to left motor"""
    def __init__(self, speed_factor=1.0, max_duty_cycle=90):
        MotorSpeed.__init__(self,
                            L_forward_pin,
                            L_backward_pin,
                            speed_factor,
                            max_duty_cycle,
                            motor_name="Left motor")


class MotorSpeedRight(MotorSpeed):
    """Inherited class specialized to left motor"""
    def __init__(self, speed_factor=1.0, max_duty_cycle=90):
        MotorSpeed.__init__(self,
                            R_forward_pin,
                            R_backward_pin,
                            speed_factor,
                            max_duty_cycle,
                            motor_name="Right motor")


# Keyboard functions
class _Getch(object):
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix(object):
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows(object):
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


#Class to read quadrature encoders
class QuadEncoder(Thread):
    """A class to read the two output of a quadrature encoder and estimate its speed through GPIO.
    ENCODER LOGIC: We assume that positive direction is when A is the leading edge. Thus, whenever we see
    a edge transition we check if the triggering pin has the same state as the non-triggering pin (trigger pin is following)
    or opposite (trigger pin is leading). For our 150RPM motor @4.5V with 120:1 gear ratio and encoder with 12CPR, we expect a max CPR of 150/60*120*12=3600.
    """
    def __init__(self,
                 A_pin,
                 B_pin,
                 updateInterval=0.01,
                 encoder_name="quadrature"):
        # Init the parent Thread class
        Thread.__init__(self)
        # Save the pins privately
        self.A_pin = A_pin
        self.B_pin = B_pin
        self.updateInterval = updateInterval  # Update interval in seconds
        self.encoder_name = encoder_name
        self.count = 0
        self.velocity = 0
        self.bStopFlag = False
        # Set pins as input
        if IS_RPI:
            GPIO.setup(self.A_pin, GPIO.IN)
            GPIO.setup(self.B_pin, GPIO.IN)
            # Get the initial pin states
            self.A_state = GPIO.input(self.A_pin)
            self.B_state = GPIO.input(self.B_pin)
            # Add interrupts for the encoder reading
            time.sleep(0.01)
            GPIO.add_event_detect(self.A_pin,
                                  GPIO.BOTH,
                                  callback=self.A_callback)
            time.sleep(0.01)
            GPIO.add_event_detect(self.B_pin,
                                  GPIO.BOTH,
                                  callback=self.B_callback)
        else:
            self.A_state = True
            self.B_state = True
            print('Encoder "%s" initialized' % encoder_name)

    def A_callback(self, channel):
        self.A_state = GPIO.input(self.A_pin)
        if self.A_state == self.B_state:
            self.count -= 1  # A follows B
        else:
            self.count += 1  # A leads B

    def B_callback(self, channel):
        self.B_state = GPIO.input(self.B_pin)
        if self.A_state == self.B_state:
            self.count += 1  # B follows A
        else:
            self.count -= 1  # B leads A

    # Override the default Thread.run() function to compute the encoder speed as counts/second
    def run(self):
        lastUpdateTime = time.time(
        ) - 1.00  #add an offset so we don't divide by zero
        while not self.bStopFlag:
            currentTime = time.time()
            self.velocity = float(self.count) / (currentTime - lastUpdateTime)
            self.count = 0  # Note: If there is threading issues we may need a lock for modifying this variable
            lastUpdateTime = currentTime
            time.sleep(self.updateInterval)

    # Return the velocity in counts/seconds
    def getVelocity(self):
        return self.velocity

    # Function to update the interval
    def setInterval(self, newInterval):
        self.updateInterval = newInterval

    # Function to stop thread
    def stop(self):
        self.bStopFlag = True


# Specialized class for left and right encoders
class QuadEncoderRight(QuadEncoder):
    """Specialized class to create a right encoder"""
    def __init__(self, updateInterval=0.01):
        QuadEncoder.__init__(self, R_encoder_A, R_encoder_B, updateInterval,
                             "Right Encoder")


class QuadEncoderLeft(QuadEncoder):
    """Specialized class to create a left encoder"""
    def __init__(self, updateInterval=0.01):
        QuadEncoder.__init__(self, L_encoder_A, L_encoder_B, updateInterval,
                             "Left Encoder")


# CSV reading
def read_two_columns_csv(filename):
    """
    Read a CSV (Comma Separated Values) file with numerical values,
    and return a list of lists with the contents of the first two columns of the file.
    If there is an error in opening the file, the returned list is empty.
    If a row in the file contains less than two, it is skipped.
    """
    import numpy as np

    pair_list = []
    with open(filename, 'r') as file_id:
        #use one of NumPy functions to load the data into an array
        data = np.genfromtxt(file_id, delimiter=',')
        #iterate over the rows
        for row in data:
            if len(row) >= 2:
                #append the content of the first two columns to the list
                pair_list.append([row[0], row[1]])
    return pair_list
