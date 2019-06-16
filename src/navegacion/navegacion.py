#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Author Berna
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Vector3,PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import time
import visualization_msgs
from visualization_msgs.msg import Marker
import numpy as np
import math

"""
    Codigo de navegacion aplicando el principio de campos potenciales
"""

class navegacion:
    """
        Constructor, donde creamos nuestros publicadores y suscriptores

    """    
    def __init__(self,start_x,start_y,width,height):
        rospy.Subscriber("/odom",Odometry,self.get_odom)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal)
        rospy.Subscriber("/piso_publicador",OccupancyGrid,self.grid) #Se suscribe a los datos del OccupancyGrid

        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.mapa=None #guarda un np.array
        self.once=False
        self.x=start_x
        self.y=start_y
        self.punto_r=[self.x,self.y]
        self.goal=np.array([[4.0, 2.97]])
        self.width=width #X
        self.height=height #Y
        self.vel_lineal=0
        self.vel_angular = 0          # RADIANES
        

    """
        descompone los valores del topico poseStamped y los asignamos a una variable global.
        Se cambia la lectura de la orientacion de la kobuki a quaterniones 
    """
    def get_odom(self,msg):
        self.pose=msg
        #COORDENADAS EN EL ENTORNO REAL
        self.x=msg.pose.pose.position.x 
        self.y=msg.pose.pose.position.y 
        orientation_q= msg.pose.pose.orientation
        orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (self.roll,self.pitch,self.yaw)=euler_from_quaternion(orientation_list)

    """
        Definimos el valor del objetivo dentro del grid
    """
    def _goal(self,msg):
        self.goal[0][0] = msg.pose.pose.position.x
        self.goal[0][1] = msg.pose.pose.position.y
    
    """
    Guardamos el Mapa en una matriz 
    """
    def grid(self,msg):

        mapa=msg.data
        mapa=np.array(mapa)
        mapa=mapa.reshape(self.height,self.width)
        self.mapa=mapa
        self.navega()

    """
    Dibuja un vector: 

    """
    def markerVector(self,id,vector,position,name,pub):
        marker = Marker ()
        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now ()
        marker.ns = name
        marker.id = id
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

        start.x = self.x  # position[0]
        start.y = self.y   # position[1]
        start.z =0.5       # position[2]
        end.x=start.x+vector[0][0]
        end.y=start.y+vector[0][1]
      
        marker.points.append(start)
        marker.points.append(end)
        
        pub.publish(marker)
        
    """
        Nos encargamos de unir todos nuestros calculos
        respecto a los campos potenciales para que logre avanzar y no estrellarse
        con obstaculos
    """
    def navega(self):
        cons = 1
        x_grid=int(math.floor(self.x*3.33))
        y_grid=int(math.floor(self.y*3.33))

        punto_r=[x_grid,y_grid]#[8,1]
        pto_goal=self.goal#[22,9]

        f_obs = self.fuerzas_repTotal(punto_r)  # Valores de entorno real
        f_goal = self.fuerzas_repGoal(punto_r, pto_goal)  # Valores de entorno real
        vec_resultante = cons * (f_obs + f_goal)
        vel_lineal = self.producto_punto(vec_resultante, self.orientacion_kobuki() )       # Velocidad lineal
        ortogonal=self.v_ortogonal(self.orientacion_kobuki())
        vel_angular=self.producto_punto(vec_resultante,ortogonal)

        self.vel_lineal += vel_lineal
        self.vel_angular += vel_angular
       
        print "x:", self.x, "\ty:", self.y
        print "Goal:", self.goal
        print "x_grid:", x_grid,"\ty_grid:",y_grid
        print "f_obstaculos:\t",f_obs
        print "f_goal:", f_goal
        print "vec_resultante:\t", vec_resultante
        print "orientacion_kobuki\t", self.orientacion_kobuki()
        print "ortogonal_kobuki:\t", ortogonal
        print "Velocidad lineal:\t", vel_lineal 
        print "Velocidad angular:\t", vel_angular

        print "***************************************************"
        vec_res_pub = rospy.Publisher ("vector_resultante", visualization_msgs.msg.Marker)
        self.markerVector(1,vec_resultante, np.zeros(2),"vector_resultante",vec_res_pub)
        vec_force_pub = rospy.Publisher ("fuerza_obstaculos", visualization_msgs.msg.Marker)
        self.markerVector(2,f_obs, np.zeros(2),"fuerza_obstaculos",vec_force_pub)
        vec_goal_pub = rospy.Publisher ("fuerza_goal", visualization_msgs.msg.Marker)
        self.markerVector(3,f_goal, np.zeros(2),"vector_goal",vec_goal_pub)
        # if self.goal[0] != x_grid and self.goal[1] != y_grid:
        self.move()
        
        rospy.sleep(0.01)



    """
        Movemos al robot
    """
    def move(self):
        cons_a=0.2
        cons_l = 0.2
        vel_msg = Twist()
        # Publish the velocity
        self.vel_lineal *= cons_l #Radianes
        self.vel_angular *= cons_a #Radianes

        if self.vel_lineal > 1:
            self.vel_lineal =1
        elif self.vel_lineal < -1:
            self.vel_lineal=-1

        if self.vel_angular > np.radians(360):
            self.vel_angular = np.radians(360)
        elif self.vel_angular < -np.radians(360):
            self.vel_angular = -np.radians(360)

        # Angular velocity in the z-axis.
        vel_msg.linear.x = self.vel_lineal       # velocidad
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = - self.vel_angular

        self.velocity_publisher.publish(vel_msg)
        print "velocidad lineal real:\t", self.vel_lineal
        print "Velocidad angular real:\t", self.vel_angular, "\n\n"

    """
     De un punto en el grid lo convierte a las medidas fisicas originales
    """
    def to_real(self, pto):
        p = np.array([pto])
        p = p*0.333
        return p

    """
        Realiza el producto punto
    """
    def producto_punto(self, f, k):
       
        res=f[0][0]*k[0] + f[0][1]*k[1]
        return res

    """
    Obtenemos el vector ortogonal (perpendicular) al que nos da la orientacion
    de Kobuki, esto para proyectarlo y poder obtener la velocidad
    angular
    """
    def v_ortogonal(self,v):
        return np.array([v[1], -v[0]])


    """
    Obtenemos la orientacion del robot con un vector unitario respecto a la lectura del odometro. 
    return coordenadas(x,y)
    """
    def orientacion_kobuki(self):
        y=np.sin(self.yaw)
        x=np.cos(self.yaw)
        return np.array([x,y])


    """
    Calculamos la fuerza  de atraccion entre la ubicacion en el punto del robot p_r 
    y la ubicacion de la meta p_i
    return la fuerza de atraccion entre p_r y p_i  
    """

    def fuerzas_repGoal(self,punto_r,pto_goal):
        cons=1.5
        #d=self.distancia(punto_r,pto_goal)
        n = ((self.goal[0][0]-self.x)**2)+((self.goal[0][1]-self.y)**2)       #Distancia
        d = math.sqrt(n)

        f=cons * self.fuerza_atraccion(self.to_real(punto_r),self.to_real(pto_goal),d)
        return f

    """
    calculamos la distancia entre dos puntos 
        punto_r posicion del robot
        punto_i posicion del goal
        
    
    """
    def distancia(self,punto_r,punto_i):
        n=((punto_i[0][0]-self.x)**2)+((punto_i[0][1]-self.y)**2)
        return math.sqrt(n)

    """
    Calcula fuerza repulsiva
    """
    def fuerza_repulsiva(self,punto_r,punto_i,distancia):
        a=np.array([[self.x,self.y]])- punto_i

        f=np.true_divide(a, distancia**3)      
        
        return f

    """
    Calcula fuerza de atraccion
    """
    def fuerza_atraccion(self,punto_r,punto_goal,distancia):
        cons=1
        
        a=self.goal-np.array([[self.x,self.y]])
        f=a * cons
        
        return f

    """
    Buscamos colisiones
    
    Colisiones en Todos los puntos cardinales(sensores), respecto al mapa.
    Dada la colision en el punto p_i en cada direccion del robot que se encuentra en el punto p_r 
    calculamos la distancia d entre p_r y p_i y despues calculamos la fuerza repulsiva f_i con p_r, p_i 
    y distancia d.  

    return vector con la suma de las i-esimas fuerzas repulsivas
    """
    def fuerzas_repTotal(self,punto_r):
        cons=1.3
       
        res=np.array([[0.0,0.0]])
        punto_i=self.colision_N(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f = cons * self.fuerza_repulsiva(self.to_real(punto_r), self.to_real(punto_i), d)
        res+=f

        punto_i=self.colision_NE(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f = cons * self.fuerza_repulsiva(self.to_real(punto_r), self.to_real(punto_i), d)
        res+=f

        punto_i=self.colision_E(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f= cons * self.fuerza_repulsiva(self.to_real(punto_r),self.to_real(punto_i),d)
        res+=f

        punto_i=self.colision_S(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f= cons * self.fuerza_repulsiva(self.to_real(punto_r),self.to_real(punto_i),d)
        res+=f

        punto_i=self.colision_SE(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f= cons * self.fuerza_repulsiva(self.to_real(punto_r),self.to_real(punto_i),d)
        res+=f

        punto_i=self.colision_SO(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f= cons * self.fuerza_repulsiva(self.to_real(punto_r),self.to_real(punto_i),d)
        res+=f

        punto_i=self.colision_O(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f= cons * self.fuerza_repulsiva(self.to_real(punto_r),self.to_real(punto_i),d)
        res+=f

        punto_i=self.colision_NO(punto_r)
        d = self.distancia(punto_r, self.to_real(punto_i))
        f= cons * self.fuerza_repulsiva(self.to_real(punto_r),self.to_real(punto_i),d)
        res+=f

        return res

    """
        REVISAMOS COLISIONES HACIA LOS OCHO LADOS QUE NOS PERMITE LA NATURALEZA DEL GRID.
        punto_r : Las coordenadas actuales de la kobuki. 

    """
    def colision_N(self,punto_r):#(x,y)
        x=punto_r[0]
        y_temp=punto_r[1]
        while x < self.width and y_temp < self.height:
            if self.mapa[y_temp][x] == 100:
                return ([x,y_temp-1])
            y_temp+=1
        return [x,self.height-1]

    def colision_S(self,punto_r):
        x=punto_r[0]
        y_temp=punto_r[1]

        while x < self.width and y_temp >=0:
            if self.mapa[y_temp][x]==100:
                return [x,y_temp+1]
            y_temp-=1
        return [x,0]

    def colision_E(self,punto_r):#(x,y)
        x_temp=punto_r[0]
        y=punto_r[1]
        while x_temp < self.width and y < self.height:
            if self.mapa[y][x_temp] == 100:
                return ([x_temp-1,y])
            x_temp+=1
        return [self.width-1,y]

    def colision_O(self,punto_r):
        x_temp=punto_r[0]
        y=punto_r[1]
        while x_temp > 0 and y < self.height:
            if self.mapa[y][x_temp] == 100:
                return ([x_temp+1,y])
            x_temp-=1
        return [0,y]

    def colision_NE(self,punto_r):
        x_temp=punto_r[0]
        y_temp=punto_r[1]
        while x_temp < self.width and y_temp < self.height:
            if self.mapa[y_temp][x_temp] == 100:
                return [x_temp-1, y_temp-1]
            x_temp+=1
            y_temp+=1
        return [self.width-1, self.height-1]

    def colision_SE(self,punto_r):
        x_temp=punto_r[0]
        y_temp=punto_r[1]
        while x_temp < self.width and y_temp >= 0:
            if self.mapa[y_temp][x_temp] == 100:
                return [x_temp-1,y_temp+1]
            x_temp+=1
            y_temp-=1
        return [self.width-1,0]

    def colision_SO(self,punto_r):
        x_temp=punto_r[0]
        y_temp=punto_r[1]
        while x_temp >= 0 and y_temp >= 0:
            if self.mapa[y_temp][x_temp] == 100:
                return [x_temp+1,y_temp+1]
            x_temp-=1
            y_temp-=1
        return [0,0]

    def colision_NO(self,punto_r):
        x_temp=punto_r[0]
        y_temp=punto_r[1]
        while x_temp >= 0 and y_temp < self.height:
            if self.mapa[y_temp][x_temp] == 100:
                return [x_temp+1,y_temp-1]
            x_temp-=1
            y_temp+=1
        return [0,self.height-1]



def main():
    x=8
    y=1
    width=24
    height=30
    
    rospy.init_node("navegacion_node",anonymous=True)
    nav=navegacion(x,y,width,height)


    #nav.show()
    rospy.spin()


if __name__ == '__main__':
    main()
