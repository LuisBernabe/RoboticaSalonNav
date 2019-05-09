#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def odometryCb(msg):
    orientation_q= msg.pose.pose.orientation
    orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
    (roll,pitch,yaw)=euler_from_quaternion(orientation_list)
    print "",[roll, pitch, yaw]
    # print "yaw:",toDegreesPos(yaw)

def toDegreesPos(yaw):
    grados=np.degrees(yaw)
    if grados < 0:
        grados=grados +360
    return grados

if __name__ == "__main__":
    rospy.init_node('quaternion_to_euler', anonymous=True) #make node
    rospy.Subscriber('odom',Odometry,odometryCb)
    rospy.spin()
