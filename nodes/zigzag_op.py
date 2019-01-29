#!/usr/bin/env python

## Node that outputs a sequence of forward-moving arches

import rospy
from geometry_msgs.msg import Twist


class ZigZagCommander():
    def __init__(self):
        self.pub = rospy.Publisher('motor_vel', Twist, queue_size=10)
        self.count = 0
        self.nbActions = 2

    def send(self):
        #Init message. It is a 3-D Twist, but we will use only two fields
        msg = Twist()

        #Fill in linear (x-axis) and angular (z-axis) velocities
        msg.linear.x = 0.5
        if self.count == 0:
            msg.angular.z = -0.5
        else:
            msg.angular.z = 0.5

        #Update count
        self.count = (self.count + 1) % self.nbActions

        #Publish
        self.pub.publish(msg)


def zigzag():
    rospy.init_node('zigzag_twist')
    #Set rate to use (in Hz)
    rate = rospy.Rate(1)
    zo = ZigZagCommander()
    while not rospy.is_shutdown():
        #Talk
        zo.send()
        #Wait until it is done
        rate.sleep()


if __name__ == '__main__':
    zigzag()
