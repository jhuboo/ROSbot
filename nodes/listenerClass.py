#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

class Listener:
    def __init__(self):
        #Use the 'chatter' topic
        rospy.Subscriber('chatter', String, self.callback)

    def callback(self,data):
        rospy.loginfo(' I heard %s', data.data)    

def listener():
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('listener',anonymous=True)
    #Create the listener object (it will setup its own callback
    lo=Listener()
    #Run until stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass

