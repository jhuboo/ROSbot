#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(' I heard %s', data.data)

def listener():
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('listener',anonymous=True)
    #Use the 'chatter' topic
    rospy.Subscriber('chatter', String, callback)
    #Run until stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

