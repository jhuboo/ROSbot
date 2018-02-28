#!/usr/bin/env python

## Node that outputs a sequence of zero-velocity commands

import rospy
from geometry_msgs.msg import Twist

class ZigZagCommander():
    def __init__(self):
        self.count=0
        self.nbActions=2;

    def send(self):
        #Init message. It is a 3-D Twist, but we will use only two fields
        #Fill in linear (x-axis) and angular (z-axis) velocities, and update count
        msg.linear.x=0.5
        if self.count==0:
            msg.angular.z=-0.5
            self.count=(self.count+1)%self.nbActions
        else:
            msg.angular.z=0.5
            self.count=(self.count+1)%self.nbActions
        #Publish
        
def zero_command():
    rospy.init_node('null_op')
    #Set rate to use (in Hz)
    rate = rospy.Rate(10)
    pub=rospy.Publisher('motor_vel', Twist, queue_size=10)
    #Prepare a Twist message with 
    msg=Twist()
    while not rospy.is_shutdown():
        #Talk
        pub.publish(msg)
        #Wait until it is done
        rate.sleep()

if __name__ == '__main__':
    zero_command()
