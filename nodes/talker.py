#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def talker():
    #Publish on the 'chatter' topic
    pub = rospy.Publisher('chatter', String, queue_size=10)
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker',anonymous='True')
    #Set rate to use (in Hz)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #Prepare string. Note that this is one of the fundamental types.
        #Usually we would need to initialize the structure of the message
        hello_str = "Hello world. The time is %s." % rospy.get_time()
        #Write to console
        rospy.loginfo(hello_str)
        #Publish
        pub.publish(hello_str)
        #Wait until it is done
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
