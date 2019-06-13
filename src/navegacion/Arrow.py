#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs
import numpy as np
from geometry_msgs.msg import Point


def markerVector(id,vector,position):
    marker = Marker ()
    marker.header.frame_id = "/base_link";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "my_namespace2";
    marker.id = id;
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.scale.x=0.1
    marker.scale.y=0.3
    marker.scale.z=0.1
    marker.color.a= 1.0
    marker.color.r = 0.33*float(id)
    marker.color.g = 0.33*float(id)
    marker.color.b = 0.33*float(id)
    (start,end)=(Point(),Point())

    start.x = position[0]
    start.y = position[1]
    start.z =0.5       # position[2]
    end.x=start.x+vector[0]
    end.y=start.y+vector[1]
    #   end.z=start.z+vector[2]

    marker.points.append(start)
    marker.points.append(end)
    print str(marker)
    return marker


rospy.init_node ('arrowsample', anonymous = True)
arrow_pub = rospy.Publisher ("visualization_arrow", visualization_msgs.msg.Marker)

while not rospy.is_shutdown():
    v=markerVector(3,np.array([-5,-3]), np.zeros(2))
    arrow_pub.publish(v)
