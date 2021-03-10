#!/usr/bin/env python
"""
    motor_command.py
    Subscribes to topics: robot_twist
    Publishes to topics : motor_speeds
"""

import rospy
import motor_command_model
import me416_utilities
from std_msgs.msg import String



def callback(msg):
    pass


def main():
    motor_left  = me416_utilities.MotorSpeedLeft()
    motor_right  = me416_utilities.MotorSpeedRight()

    # Initialize node, need to check if multiple launches allowed or not for this one
    rospy.init_node('', anonymous='True')

    rospy.Subscriber('robot_twist', geometry_msg/Twist, callback)
    pub = rospy.Publisher('motor_speeds', me416_lab/MotorSpeedsStamped, callback)



    pass


if __name__ == '__main__':
    try:
        main()
    finally:
        # Clean up
        pass
