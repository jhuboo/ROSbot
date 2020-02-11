#!/usr/bin/env python
"""Simple ROS node to publish both encoder velocities in counts/seconds.
"""

import rospy
from me416_lab.msg import MotorSpeedsStamped
import me416_utilities as mu

if __name__ == '__main__':
    rospy.init_node('encoder_node')

    # Create encoder objects
    encoderRight = mu.QuadEncoderRight()
    encoderLeft = mu.QuadEncoderLeft()

    # Prepare publisher for sending encoder messages
    pub = rospy.Publisher('encoders', MotorSpeedsStamped, queue_size=1)

    # Prep message object
    msg = MotorSpeedsStamped()

    # Set rate in Hz
    rate = rospy.Rate(60)

    try:
        encoderRight.start()
        encoderLeft.start()
        while not rospy.is_shutdown():
            msg.header.frame_id = 'encoders'
            msg.header.stamp = rospy.Time.now()
            msg.left = encoderLeft.getVelocity()
            msg.right = encoderRight.getVelocity()
            pub.publish(msg)
            rate.sleep()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        encoderLeft.stop()
        encoderRight.stop()
        encoderLeft.join()
        encoderRight.join()
