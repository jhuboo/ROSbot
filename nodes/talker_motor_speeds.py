#!/usr/bin/env python
"""Simple talker demo that published std_msgs/Strings messages to the 'chatter' topic"""

import rospy
from me416_lab.msg import MotorSpeedsStamped


def main():
    """Node setup and main ROS loop"""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker_motor_speeds')

    #Prepare publisher on the 'motor_speeds' topic
    pub = rospy.Publisher('motor_speeds', MotorSpeedsStamped, queue_size=10)

    #Prepare message object
    msg = MotorSpeedsStamped()

    #Set rate to use (in Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg.left = -1.0
        msg.right = -1.0
        msg.header.frame_id = 'base_link'
        msg.header.stamp = rospy.Time.now()
        #msg.header.seq is filled by ROS automatically

        #Write to console
        rospy.loginfo('Message updated')

        #Publish
        pub.publish(msg)

        #Wait until it is done
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
