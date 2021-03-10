""" Example of how to set attributes in ROS messages """

from geometry_msgs.msg import Twist

def twist_fill(A = Twist()):
    """Create ob A of type geometry_As/Twist, fill attributes, return A """
    A.linear.x = 1.0
    A.linear.y = 2.0
    A.linear.z = 3.0
    A.angular.x = 1.1
    A.angular.y = 2.2
    A.angular.z = 3.3

    return A

