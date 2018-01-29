#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
## Wraps the "talker" into an object, and show how to use an "internal state"

import rospy
from std_msgs.msg import String

class Talker():
    def __init__(self):
        #Init publisher on the 'chatter' topic
        self.pub=rospy.Publisher('chatter', String, queue_size=10)
        self.count=0

    def updateCount(self):
        #Count from 0 to 9 and then repeat
        self.count+=1
        if self.count>9:
            self.count=0
        #The above could be simplified as 
        #self.count=(self.count+1)%10

    def talk(self):
        self.updateCount()
        #Prepare string. Note that this is one of the fundamental types.
        #Usually we would need to initialize the structure of the message
        hello_str = "Hello world. I am counting %d." % self.count
        #Write to console
        rospy.loginfo(hello_str)
        #Publish
        self.pub.publish(hello_str)
        
def talker():
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker',anonymous='True')
    #Set rate to use (in Hz)
    rate = rospy.Rate(1)
    #to stands for tALKER oBJECT
    to=Talker()
    while not rospy.is_shutdown():
        #Talk
        to.talk()
        #Wait until it is done
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
