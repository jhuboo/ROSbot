#!/usr/bin/env python
"""
Simple repeater demo that receives std_msgs/Strings messages from the 'chatter'
 topic, modifies them, and then sends them on the 'chatter_repeated' topic
"""

import rospy
from std_msgs.msg import String


def callback(msg):
    """ Callback to receive a message and repeat it with some modifications """
    global accumulator

    #Take content from the message
    content = msg.data
    #Manipulate content (in this case, change some of the character to '*')
    content = ''.join(['*' if x in 'aeiou01234' else x for x in content])

    #Publish a new message with the modified string
    msg_repeated.data += " " + content
    # pub.publish(msg_repeated)

    #Show some logging information (optional)
    #rospy.loginfo(' I heard %s', msg.data)
    #rospy.loginfo(' I repeated %s', msg_repeated.data)


def main():
    """Node setup and main ROS loop"""
    global accumulator

    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker', anonymous='True')

    #Prepare subscriber on a topic, and publisher on another
    rospy.Subscriber('chatter', String, callback)
    pub = rospy.Publisher('chatter_repeated', String, queue_size=10)

    # Set rate using frequency (in Hz)
    rate = rospy.Rate(0.33)

    while not rospy.is_shutdown():
        pub.publish(msg_repeated)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
