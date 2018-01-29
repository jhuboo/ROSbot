
## This is a library of helpful classes and functions for the ME416 Lab.

import RPi.GPIO as GPIO
import atexit

# Set the pin numbering scheme to board numbers (as opposed to Broadcom number, see the pinout)
GPIO.setmode(GPIO.BOARD)
atexit.register(GPIO.cleanup)

# Assign variable names to the pins so you don't have to control them by number
R_forward_pin = 31
R_backward_pin = 29
L_forward_pin = 16
L_backward_pin = 18

# Motor control class
class MotorSpeed:
    """A class to control motors using PWM on all channels of an H-bridge thorugh GPIO"""
    def __init__(self,fw_pin,bw_pin,speed_offset=1.0,max_duty_cycle=90):
        #Save parameter privately
        self.speed_offset=speed_offset
        self.max_duty_cycle=90
        
        #Init pins and PWM objects
        GPIO.setup(fw_pin, GPIO.OUT)
        GPIO.setup(bw_pin, GPIO.OUT)
        self.fw_pwm=GPIO.PWM(fw_pin, 100)
        self.bw_pwm=GPIO.PWM(bw_pin, 100)
        self.fw_pwm.start(0)
        self.bw_pwm.start(0)
        
    def set_speed(self,speed):
        """ Set speed. speed=-1 is max_duty_cycle backward, speed=1 is max_duty_cycle foward, speed=0 is stop """
        duty_cycle=int(abs(speed)*self.speed_offset*self.max_duty_cycle)
        if speed<0:
            self.bw_pwm.ChangeDutyCycle(duty_cycle)
            self.fw_pwm.ChangeDutyCycle(0)
        elif speed>0:
            self.bw_pwm.ChangeDutyCycle(0)
            self.fw_pwm.ChangeDutyCycle(duty_cycle)
        else:
            self.bw_pwm.ChangeDutyCycle(0)
            self.fw_pwm.ChangeDutyCycle(0)

#Specialized motor classes
class MotorSpeedLeft(MotorSpeed):
    """Inherited class specialized to left motor"""
    def __init__(self,speed_offset=1.0,max_duty_cycle=90):
        MotorSpeed.__init__(self,L_forward_pin,L_backward_pin,speed_offset,max_duty_cycle)
        
class MotorSpeedRight(MotorSpeed):
    """Inherited class specialized to left motor"""
    def __init__(self,speed_offset=1.0,max_duty_cycle=90):
        MotorSpeed.__init__(self,R_forward_pin,R_backward_pin,speed_offset,max_duty_cycle)


# Keyboard functions
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
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


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


