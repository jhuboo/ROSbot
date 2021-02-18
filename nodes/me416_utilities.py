"""
This is a library of helpful classes and functions for the ME416 Lab. If the module is on non-RasberryPi systems (more
exactly, where the RPi module is not available), the motor commands are logged to the console
"""

from __future__ import print_function
from threading import Thread, Event
import atexit
import time
import psutil
import math
# The following are needed to retrieve the $USER variable
import os
import pwd

# This module might be used not on a RPi (i.e., without GPIO)
# The global constant will allow to know if we can use the GPIO or not
try:
    import RPi.GPIO as GPIO
except ImportError:
    IS_RPI = False
else:
    IS_RPI = True

# Check if Gazebo is running, import functions accoringly, and set global variable IS_GAZEBO:
IS_GAZEBO = False
if [proc.name() for proc in psutil.process_iter() if ('gzserver' == proc.name() and\
    proc.username() == pwd.getpwuid(os.getuid())[0])]:
    
    import rospy
    from gazebo_msgs.msg import LinkStates
    from std_msgs.msg import Float64
    from geometry_msgs.msg import Vector3Stamped
    import tf2_geometry_msgs as tr
    import transformation_utilities as tu
    import tf2_ros    
    import numpy as np
    
    IS_GAZEBO = True

# Set the pin numbering scheme to board numbers (as opposed to Broadcom number, see the pinout)
if IS_RPI:
    GPIO.setmode(GPIO.BOARD)

# Wrappers for setting pins and cleanup. These are used so that we can cleanup only when we actually set some pins (avoid warning messages)
IS_SETUP = False


def setup(pin, mode):
    global IS_SETUP
    if IS_RPI:
        GPIO.setup(pin, mode)
        IS_SETUP = True


def rpi_cleanup():
    if IS_SETUP and IS_RPI:
        GPIO.cleanup()

def rospy_cleanup():
    rospy.Publisher("/right_wheel_cmd",Float64,queue_size=10).publish(0)
    rospy.Publisher("/left_wheel_cmd",Float64,queue_size=10).publish(0)

# Register the cleanup functions to run at exit
if IS_RPI:
    atexit.register(rpi_cleanup)
elif IS_GAZEBO:
    rospy.on_shutdown(rospy_cleanup)
    
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

# Max speed measured by encoders, so that get_velocity() returns a value in [-1,1]
MAX_CPS_5V = 3600.0 # In counts per second (CPS)
# 150 RPM is the maximum speed. We convert it to rad/s:
MAX_RADS_5V = 150 * math.pi * 2 / 60.0

# Motor class
class Motor(object):
    """
    The main motor class.
    """
    def __init__(self,
                 name,
                 encoder,
                 actuator):

        self.name = name
        self.encoder = encoder
        self.actuator = actuator

    def get_velocity(self):
        return self.encoder.get_velocity()

    def set_velocity(self,velocity):
        return self.actuator.set_velocity(velocity)

class GazeboMotor(Motor):
    def __init__(self,
                 name,
                 encoder,
                 actuator):

        Motor.__init__(self,name,encoder,actuator)

class PlasticMotor(Motor):
    def __init__(self,
                 name,
                 encoder,
                 actuator):
                 
        Motor.__init__(self,name,encoder,actuator)
        

def createMotor(name="",updateInterval=0.1,speed_factor = 1.0,max_duty_cycle = 90):
    
    encoder = createEncoder(name,updateInterval)
    actuator = createActuator(name,speed_factor,max_duty_cycle)

    if IS_GAZEBO:
        return GazeboMotor(name,encoder,actuator)
    elif IS_RPI:
        return PlasticMotor(name,encoder,actuator)

    print("Initializing default motor...")
    return Motor(name+" (default)")

def createEncoder(name="",updateInterval=0.1):

    if 'left' not in name.lower() and 'right' not in name.lower():
        raise NameError("Invalid name. Include 'left' or 'right' in the name")

    if IS_GAZEBO:
        return GazeboQuadEncoder(name)
    elif IS_RPI:
        if 'right' in name.lower():
            return PlasticQuadEncoder(R_encoder_A, R_encoder_B, updateInterval,name)
        elif 'left' in name.lower():
            return PlasticQuadEncoder(L_encoder_A, L_encoder_B, updateInterval,name)
    
    print("Initializing default encoder...")
    return Encoder(name+" (default)")

def createActuator(name="",speed_factor=1.0,max_duty_cycle=90):
    
    if 'left' not in name.lower() and 'right' not in name.lower():
        raise NameError("Invalid name. Include 'left' or 'right' in the name")

    if IS_GAZEBO:
        if "right" in name.lower():
            # A small patch to flip the direction of the robot. Should fix it in the xacro file.
            return GazeboMotorActuator(name,speed_factor,max_duty_cycle,"left")
        elif "left" in name.lower():
            return GazeboMotorActuator(name,speed_factor,max_duty_cycle,"right")
    elif IS_RPI:
        if 'right' in name.lower():
            return PlasticMotorActuator(R_forward_pin,
                                        R_backward_pin,
                                        speed_factor,
                                        max_duty_cycle,
                                        motor_name=name)
        elif 'left' in name.lower():
            return PlasticMotorActuator(L_forward_pin,
                                        L_backward_pin,
                                        speed_factor,
                                        max_duty_cycle,
                                        motor_name=name)
    return MotorActuator(name+" (default)")

# Motor control class
class MotorActuator(object):
    def __init__(self,motor_name="default actuator",speed_factor=1.0,max_duty_cycle=90):
        self.speed_factor = speed_factor
        self.max_duty_cycle = max_duty_cycle
        self.name = motor_name

    def set_velocity(self,velocity):
        # Dummy set_velocity() function, to avoid NotImplemented errors
        print("Setting velocity of dummy motor to: ", velocity)

class GazeboMotorActuator(MotorActuator):
    def __init__(self,motor_name="right gazebo actuator",speed_factor=1.0,max_duty_cycle=90,wheel="right"):

        MotorActuator.__init__(self,motor_name,speed_factor,max_duty_cycle)

        # Velocity publisher
        self.vel_publisher = rospy.Publisher("/{}_wheel_cmd".format(wheel),Float64,queue_size=1)

    def set_velocity(self,velocity):
        """Publish desired wheel velocity, between [-1,1]"""
        vel_msg = Float64()
        vel_msg.data = min(1,max(-1,-velocity))
        self.vel_publisher.publish(vel_msg)

class PlasticMotorActuator(MotorActuator):
    """A class to control motors using PWM on all channels of an H-bridge thorugh GPIO"""
    def __init__(self,
                 fw_pin,
                 bw_pin,
                 speed_factor=1.0,
                 max_duty_cycle=90,
                 motor_name="right plastic actuator"):

        MotorActuator.__init__(self,motor_name,speed_factor,max_duty_cycle)

        #Init pins and PWM objects

        setup(fw_pin, GPIO.OUT)
        setup(bw_pin, GPIO.OUT)
        self.fw_pwm = GPIO.PWM(fw_pin, 100)
        self.bw_pwm = GPIO.PWM(bw_pin, 100)
        self.fw_pwm.start(0)
        self.bw_pwm.start(0)

    def set_velocity(self, speed):
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

        self.bw_pwm.ChangeDutyCycle(duty_cycle_bw)
        self.fw_pwm.ChangeDutyCycle(duty_cycle_fw)


class Encoder(object):
    def __init__(self,encoder_name="default encoder",updateInterval=0.1,conversion_factor=1.0):

        self.updateInterval = updateInterval
        self.name = encoder_name
        self.velocity = 0.0

        # conversion_factor will make velocity readings to be in the range [-1,1].
        # Gazebo reads rad/s, and our max speed in rad/s will be 150rpm * 2*pi/60
        # ROSBot reads in counts per second (CPS), and our max speed will be 3600 CPS
        self.conversion_factor = conversion_factor
        

    def update_velocity(self,velocity):
        self.velocity = velocity

    def get_velocity(self):
        return self.velocity / self.conversion_factor

class GazeboQuadEncoder(Encoder):
    def __init__(self,name):
        Encoder.__init__(self,name,conversion_factor = MAX_RADS_5V)

        self.wheel_radius = rospy.get_param("/wheel_radius",1.0)
        self.axis_distance = rospy.get_param("/axis_distance",1.0)

        # Index corresponding to the wheel --> They should be 1/0, not 0/1. Right now it's a temporary patch
        self.encoder_index = 0 if "left" in name.lower() else 1 if "right" in name.lower() else None

        # Subscriber to get robot velocity
        self.gazebo_subscriber = rospy.Subscriber("/gazebo/link_states",LinkStates,self.gazebo_callback)
        # Transform buffers to look up required transformations for robot velocity readings
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


    def gazebo_callback(self,msg):
        """
        When gazebo is running, retrieve the wheel angular velocity.
        It is in global frame of reference! We need to convert it to the robot local frame (i.e. base link)
        """

        try:
            # read angular velocity is on /odom (global) frame. Need to convert it to robot frame.
            # To do so, read the transformation from robot frame to world frame:
            transform_to_base = tu.msg_to_se3(msg.pose[1])

            # Convert linear and angular velocities of robot to robot reference frame.
            # Then, obtain angular velocities of wheels wrt. robot reference frame.
            # The base of the robot has global velocity coordinates at index msg.twist[1].
            # The rotation of transform_to_base needs to be transposed to get the world->robot transformation:
            linear_velocity  = transform_to_base[:3,:3].T.dot(np.array([msg.twist[1].linear.x,\
                                                                        msg.twist[1].linear.y,\
                                                                        msg.twist[1].linear.z]))[0]

            angular_velocity = transform_to_base[:3,:3].T.dot(np.array([msg.twist[1].angular.x,\
                                                                        msg.twist[1].angular.y,\
                                                                        msg.twist[1].angular.z]))[2]


            # Wheel angular velocities [right,left]:
            wheel_velocities = [(2*linear_velocity+angular_velocity*2*self.axis_distance)/(2*self.wheel_radius),
                                (2*linear_velocity-angular_velocity*2*self.axis_distance)/(2*self.wheel_radius)]

            self.update_velocity(-wheel_velocities[self.encoder_index])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    

#Class to read quadrature encoders
class PlasticQuadEncoder(Encoder):
    """
    A class to read the two output of a quadrature encoder and estimate its speed through GPIO.
    If the updateInterval is set to None, the velocity is computed by averaging between calls to get_velocity; otherwise, it is updates every updateInterval seconds, using a daemonic thread to update it.
    ENCODER LOGIC: We assume that positive direction is when A is the leading edge. Thus, whenever we see
    a edge transition we check if the triggering pin has the same state as the non-triggering pin (trigger pin is following)
    or opposite (trigger pin is leading). For our 150RPM motor @4.5V with 120:1 gear ratio and encoder with 12CPR, we expect a max CPR of 150/60*120*12=3600.
    """
    def __init__(self,
                 A_pin,
                 B_pin,
                 updateInterval,
                 encoder_name="quadrature"):
        Encoder.__init__(self,encoder_name,updateInterval,conversion_factor=MAX_CPS_5V)

        self.lastUpdateTime = time.time() - 0.01 # small offset to prevent dividing by 0

        # Factor to convert from rad/s to CPS:
        self.encoder_cpr = 12

        # Setup pins and routines for encoder counters
        self.count = 0
        self.A_pin = A_pin
        self.B_pin = B_pin
        
        setup(self.A_pin, GPIO.IN)
        setup(self.B_pin, GPIO.IN)
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

        if updateInterval is None:
            # Update velocity only when requested
            self.thread = None
            self.event = None
        else:
            # Init a Thread and an Event to stop it
            self.thread = Thread(target=self.run)
            self.event = Event()
            # Run the thread as daemonic, so that it will terminate when the main thread stops
            self.thread.daemon = True
            # Run thread
            self.thread.start()


    def A_callback(self, channel):
        self.A_state = GPIO.input(self.A_pin)
        if self.A_state == self.B_state:
            self.count -= 1.0  # A follows B
        else:
            self.count += 1.0  # A leads B

    def B_callback(self, channel):
        self.B_state = GPIO.input(self.B_pin)
        if self.A_state == self.B_state:
            self.count += 1.0  # B follows A
        else:
            self.count -= 1.0  # B leads A

    # Thread's run() function to compute the encoder speed as counts/second
    def run(self):
        while not self.event.is_set():
            self.update_velocity()
            time.sleep(self.updateInterval)

    def update_velocity(self):
        currentTime = time.time()
        dt = currentTime - self.lastUpdateTime
        # Calculate the velocity
        # NOTE: We can use a low-pass filter or linear estimator, but it requires tuning when we change the sample time interval
        # To avoid tuning, we assume that the Pi is does not miss any ticks and the self.count is perfect along with the interval
        # measure time dt (this works well enough for the currently spec'ed parts for year 2021)
        self.velocity = self.count/dt
	    # Reset counter to 0
        self.count = 0.0
        self.lastUpdateTime = currentTime

    # Return the velocity in counts/seconds
    def get_velocity(self):
        # print("Getting speed from {}".format(self.name))
        if self.updateInterval is None:
            self.update_velocity()
        return self.velocity

    # Function to update the interval
    def set_interval(self, newInterval):
        self.updateInterval = newInterval

    # Function to stop thread
    def stop(self):
        self.event.set()

    # Destructor
    def __del__(self):
        """ Destructor: ask thread to stop """
        self.event.set()


# # Specialized class for left and right encoders
# class PlasticQuadEncoderRight(PlasticQuadEncoder):
#     """Specialized class to create a right encoder"""
#     def __init__(self, updateInterval=0.1):
#         super().__init__(self, R_encoder_A, R_encoder_B, updateInterval,
#                              "Right Encoder")


# class PlasticQuadEncoderLeft(PlasticQuadEncoder):
#     """Specialized class to create a left encoder"""
#     def __init__(self, updateInterval=0.1):
#         super().__init__(self, L_encoder_A, L_encoder_B, updateInterval,
#                              "Left Encoder")



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
