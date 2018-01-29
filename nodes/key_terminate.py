#!/usr/bin/env python
#
## Simple script that waits for the key 'q' and then terminates

import rospy
import me416_utilities as mu

def key_op():
    rospy.init_node('key_emergency_switch')
    rate = rospy.Rate(50)
    getch = mu._Getch()
    print("     Press 'q' to quit.\n")
    while not rospy.is_shutdown():
        key = getch()
        if key == 'q':
            print("")
            rospy.loginfo("Shutdown initiated")
            rospy.signal_shutdown("Shutting down initiated by key_emergency_switch")
        rate.sleep()

if __name__ == '__main__':
    try:
        key_op()
    #except rospy.ROSInterruptException:
    #    pass
    finally:
        pass
