#!/usr/bin/env python
"""Simple talker demo that published std_msgs/Strings messages to the 'chatter' topic"""

import rospy
from std_msgs.msg import String


def main():
    """Node setup and main ROS loop"""
    #Publish on the 'chatter' topic

    pub = rospy.Publisher('chatter', String, queue_size=10)
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker', anonymous='True')

    #Prepare message object
    msg = String()

    #Set rate to use (in Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg.data = 'I am %s. The time is %.2f.' % [
            rospy.get_name(), rospy.get_time()
        ]
        #Write to console
        rospy.loginfo(msg.data)
        #Publish
        pub.publish(msg.data)
        #Wait until it is done
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
