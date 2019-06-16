#!/usr/bin/env python
#9 cuadritos de ancho
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

"""
    Clase que crea un publicador Marker simulando una mesa
    author: Berna
"""

class Mesa(object):
    """
    Constructor que tiene 3 parametros:
        index: Funciona como identificador de la mesa
        x_val: coordenada x en el plano donde se encontrara
        y_val: coordenada y en el plano donde se encontrara 
        z_val: coordenada z en el plano donde se encontrara

    Los argumentos los toma del archivo launch
    """
    def __init__(self,index,x_val,y_val,z_val):
        self.marker_publicador=rospy.Publisher("/marker_mesa_"+str(index),Marker,queue_size=1)
        self.rate=rospy.Rate(1)
        self.init_marker(index,x_val,y_val,z_val)

    def init_marker(self,index,x_val,y_val,z_val):
        self.marker_obj=Marker()
        self.marker_obj.header.frame_id="/odom"
        self.marker_obj.header.stamp=rospy.get_rostime()
        self.marker_obj.ns="mesa"
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
        self.marker_obj.scale.x=9.0
        self.marker_obj.scale.y=2.0
        self.marker_obj.scale.z=2.0

        self.marker_obj.color.r=0.5
        self.marker_obj.color.g=0.5
        self.marker_obj.color.b=0.5

        self.marker_obj.color.a=1.0

        self.marker_obj.lifetime=rospy.Duration(0)




    def start(self):
        while not rospy.is_shutdown():
            self.marker_publicador.publish(self.marker_obj)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("mesa_marker_node",anonymous=True)

    idx=rospy.get_param('~num_mesa')

    x=rospy.get_param('~x')
    y=rospy.get_param('~y')
    z=rospy.get_param('~z')
    markerBasic_obj=Mesa(idx,x,y,z) #(idx,x,y,z)

    #markerBasic_obj=Mesa(0,0,0,0) #(idx,x,y,z)
    try:
        markerBasic_obj.start()
    except rospy.RosInterruptException:
        pass
