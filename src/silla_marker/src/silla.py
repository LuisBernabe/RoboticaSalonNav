#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class Silla(object):
    def __init__(self,index,x_val,y_val,z_val):
        self.marker_publicador=rospy.Publisher("/marker_silla_"+str(index),Marker,queue_size=1)
        self.rate=rospy.Rate(1)
        self.init_marker(index,x_val,y_val,z_val)

    def init_marker(self,index,x_val,y_val,z_val):
        self.marker_obj=Marker()
        self.marker_obj.header.frame_id="/odom" #base_link
        self.marker_obj.header.stamp=rospy.get_rostime()
        self.marker_obj.ns="silla"
        self.marker_obj.id=index
        self.marker_obj.type=Marker.CUBE
        self.marker_obj.action=Marker.ADD

        my_point=Point()
        my_point.x=x_val
        my_point.y=y_val
        my_point.z=z_val
        self.marker_obj.pose.position=my_point

        self.marker_obj.pose.orientation.x=0
        self.marker_obj.pose.orientation.y=0
        self.marker_obj.pose.orientation.z=0
        self.marker_obj.pose.orientation.w=0
        self.marker_obj.scale.x=2.0
        self.marker_obj.scale.y=1.0
        self.marker_obj.scale.z=1.0

        self.marker_obj.color.r=0.0
        self.marker_obj.color.g=1.0
        self.marker_obj.color.b=0.0

        self.marker_obj.color.a=1.0

        self.marker_obj.lifetime=rospy.Duration(0)




    def start(self):
        while not rospy.is_shutdown():
            self.marker_publicador.publish(self.marker_obj)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("silla_marker_node",anonymous=True)

    idx=rospy.get_param('~num_silla')

    x=rospy.get_param('~x')
    y=rospy.get_param('~y')
    z=rospy.get_param('~z')
    markerBasic_obj=Silla(idx,x,y,z) #(idx,x,y,z)
    #markerBasic_obj=Silla(0,0,0,0) #(idx,x,y,z)

    try:
        markerBasic_obj.start()
    except rospy.RosInterruptException:
        pass
