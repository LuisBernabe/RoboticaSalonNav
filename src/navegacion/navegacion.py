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



class navegacion:

    def __init__(self,start_x,start_y,width,height,coordenadas,init_orientation):
        rospy.Subscriber("/odom",Odometry,self.get_odom)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal)
        rospy.Subscriber("/piso_publicador",OccupancyGrid,self.grid) #Se suscribe a los datos del OccupancyGrid
        self.arrow_publisher=rospy.Publisher("/marker_arrow",Marker,queue_size=1)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.mapa=None #guarda un np.array
        self.once=False
        self.x=start_x
        self.y=start_y
        self.punto_r=[self.x,self.y]
        self.goal=np.array([[4.0, 2.97]])
        self.width=width #X
        self.height=height #Y
        self.orientacion=init_orientation
        self.coordenadas=coordenadas
        self.coordenadas=self.genera_coord()
        self.vel_lineal=0
        self.vel_angular = 0          # RADIANES
        #print "coordenadas",self.coordenadas

    def get_odom(self,msg):
        self.pose=msg
        #COORDENADAS EN EL ENTORNO REAL
        self.x=msg.pose.pose.position.x #* 3.33
        self.y=msg.pose.pose.position.y #* 3.33
        orientation_q= msg.pose.pose.orientation
        orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (self.roll,self.pitch,self.yaw)=euler_from_quaternion(orientation_list)

    def _goal(self,msg):
        self.goal[0][0] = msg.pose.pose.position.x
        self.goal[0][1] = msg.pose.pose.position.y
    """
    Guardamos el Mapa
    """
    def grid(self,msg):

        mapa=msg.data
        mapa=np.array(mapa)
        mapa=mapa.reshape(self.height,self.width)
    #    self.set_mapa(mapa)
        self.mapa=mapa
    #def set_mapa(self,mapa):
    #    self.mapa=mapa
        self.navega()

    def genera_coord(self):
        nw_coor=["NO","SO","SE","NE"]
        res=[]
        #print "C: ",self.coordenadas
        for idx,c in enumerate(self.coordenadas):
            coord=c[1]+0.75

            if coord > 3.0 :
                coord=3.0-coord
                coord=-3.0-coord
            #else:

            res+=[c]+[[nw_coor[idx],coord]]
            #rospy.sleep(0.2)
        self.coordenadas=res
        return res

    def make_arrow_points_marker(self,scale, tail, tip, idnum):
    # make a visualization marker array for the occupancy grid
        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = '/base_link'
        m.header.stamp = rospy.Time.now()
        m.ns = 'points_arrows'
        m.id = idnum
        m.type = Marker.ARROW
        m.pose.orientation.y = 0
        m.pose.orientation.w = 1
        m.scale = scale
        m.color.r = 0.2
        m.color.g = 0.5
        m.color.b = 1.0
        m.color.a = 0.7
        m.pose.position=tail
        #m.points =  tail#, tip ]
        self.arrow_publisher.publish(m)
        #print "Flecha ---->"
        #return m

        #scale = Vector3(2,4,0.69)
        #marker_pub.publish(make_arrow_points_marker(scale,Point(0,0,0), Point(3,0,0), 3))

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
        #self.vel_angular+=ortogonal
        #giro =  (giro - inclinacion_robot)*0.33
        print "x:", self.x, "\ty:", self.y
        print "Goal:", self.goal
        print "x_grid:", x_grid,"\ty_grid:",y_grid
        print "f_obstaculos:\t",f_obs
        print "f_goal:", f_goal# self.to_real(self.goal)
        print "vec_resultante:\t", vec_resultante
        print "orientacion_kobuki\t", self.orientacion_kobuki()
        print "ortogonal_kobuki:\t", ortogonal
        print "Velocidad lineal:\t", vel_lineal  # vec_resultante
        print "Velocidad angular:\t", vel_angular

        print "***************************************************"

        # if self.goal[0] != x_grid and self.goal[1] != y_grid:
        self.move()

        # self.orientation=self.redondea_orientacion() #Actualiza la orientacion
        rospy.sleep(0.01)
        # print self.orientation



    def move(self):
        cons_a=0.2
        cons_l = 0.2
        vel_msg = Twist()
        # Publish the velocity
        self.vel_lineal *= cons_l
        self.vel_angular *= cons_a

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

    # De un punto en el grid lo convierte a las medidas fisicas originales

    def to_real(self, pto):
        p = np.array([pto]);
        p = p*0.333       # np.true_divide(p,0.3)
        return p


    """
    Definimos el giro del robot, ya sea en sentido horario(-) o antiHorario(+)
    return giro del robot en grados
    """
    def giro_robot(self, giro):
        pos_actual = self.toDegreesPos(self.yaw)
        if pos_actual >giro:
            # antiHorario
            anti_horario=(360-pos_actual)+giro
            sentido_horario=pos_actual-giro
            if anti_horario < sentido_horario:
                return anti_horario
            else:
                return -sentido_horario
        else:
            return giro -pos_actual

    def producto_punto(self, f, k):
        #Obtenemos producto punto
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
    return coordenadas(x,y)
    """
    def orientacion_kobuki(self):
        y=np.sin(self.yaw)
        x=np.cos(self.yaw)
        return np.array([x,y])


    """
    Calculamos fuerzas para las coordenadas meta
    """

    def fuerzas_repGoal(self,punto_r,pto_goal):
        cons=1.5
        #d=self.distancia(punto_r,pto_goal)
        n = ((self.goal[0][0]-self.x)**2)+((self.goal[0][1]-self.y)**2)       #Distancia
        d = math.sqrt(n)

        f=cons * self.fuerza_atraccion(self.to_real(punto_r),self.to_real(pto_goal),d)
        return f

    def magnitud(self,vec_resultante):
        # print "def magnitud()"
        a=vec_resultante**2
        # print "\t", a
        a=a[0][0] + a[0][1]
        # print "\t", a
        #a=abs(a)
        return math.sqrt(a)

    """
    calcularmos Distancia
    """
    def distancia(self,punto_r,punto_i):
        # punto_r=self.to_real(punto_r)
        #punto_i=self.to_real(punto_i)
        n=((punto_i[0][0]-self.x)**2)+((punto_i[0][1]-self.y)**2)
        return math.sqrt(n)

    #def magnitud(self,distancia):
    #    return float(1/distancia)

    """
    Calcula fuerza repulsiva, al final lo convierte en positivo ya que
    la repele
    """
    def fuerza_repulsiva(self,punto_r,punto_i,distancia):
        a=np.array([[self.x,self.y]])- punto_i

        f=np.true_divide(a, distancia**3)       #distancia**3
        #f=np.absolute(f)
        return f#.tolist()

    """
    Calcula fuerza repulsiva, al final lo convierte en negativo
    ya que se atraen
    """
    def fuerza_atraccion(self,punto_r,punto_goal,distancia):
        cons=1
        #a=punto_goal-punto_r
        a=self.goal-np.array([[self.x,self.y]])
        f=a * cons
        #f=np.true_divide(a, distancia)
        #if f[0][0] > 0:
        #    f[0][0] = -f[0][0]
        #elif f[0][1] > 0:
        #    f[0][1] = -f[0][1]

        return f#.tolist()



    def tangente(self,vec_resultante):#Nos dara el angulo para la direccion
        return vec_resultante[0][1]/vec_resultante[0][0]

    def direccion_vector(self,vec_resultante):
        d=vec_resultante[0][1]/vec_resultante[0][0]
        dir=np.arctan(d) #En radianes
        dir=self.toDegreesPos(dir)
        #print "Direccion, en grados: \t",dir
        return dir

    """
    Buscamos colisiones
    """
    # Colisiones en Todos los puntos cardinales(sensores), respecto al mapa
    # return vector con la fuerza repulsiva
    def fuerzas_repTotal(self,punto_r):
        cons=1.3
        # punto_r_ar= np.array([punto_r])
        # print "colision_N",punto_i
        # print "Distancia_N",d
        # print "fuerza_repulsiva", f

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


    def colision_N(self,punto_r):#(x,y)
        x=punto_r[0]
        y_temp=punto_r[1]+1
        while x < self.width and y_temp < self.height:
            if self.mapa[y_temp][x] == 100:
                return ([x,y_temp-1])
            y_temp+=1
        return [x,self.height-1]

    def colision_S(self,punto_r):
        x=punto_r[0]
        y_temp=punto_r[1]-1

        while x < self.width and y_temp >=0:
            if self.mapa[y_temp][x]==100:
                return [x,y_temp+1]
            y_temp-=1
        return [x,1]

    def colision_E(self,punto_r):#(x,y)
        x_temp=punto_r[0]+1
        y=punto_r[1]
        while x_temp < self.width and y < self.height:
            if self.mapa[y][x_temp] == 100:
                return ([x_temp-1,y])
            x_temp+=1
        return [self.width-1,y]

    def colision_O(self,punto_r):
        x_temp=punto_r[0]-1
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


    """
    Convertimos radianes en grados positivos.
    El valor se nos da en radianes (de 0-3.14 y de -3.14 -0) i.e de (-pi, pi)
    el valor dado se pasa a grados y en caso de de que sea negativos se convierten
    a grados positivos
    """
    def toDegreesPos(self,yaw):
        grados=np.degrees(yaw)
        if grados < 0:
            grados=grados +360
        return grados


            #Si devuelve None es necesario rotar hacia cualquier lado un poco para que encuentre la coordenadas
#            rospy.sleep(0.05)
            #cons-=0.2



    """
    Relacion entre el odometro y la orientacion respecto al mapa fisico antes de empezar
    cualquier movimiento (No logra funcionar, aun)
    """
    def first_pos(self):
        if not self.once:
            self.once=True
            rospy.Subscriber("/odom",Odometry,self.pos)
        else:
            pass

    def pos(self,msg):
        orientation_q= msg.pose.pose.orientation
        orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (roll,pitch,first_yaw)=euler_from_quaternion(orientation_list)
        self.set_firstPos(first_yaw)

    def set_firstPos(self,yaw):
        self.yaw_inicial=yaw
        print self.yaw_inicial

def main():
    x=8
    y=1
    orientacion_inicial="O"
    width=24
    height=30
    yaw_inicial=-1.5310
    coordenadas=[["N",1.5179],["O",3.0931],["S",-1.5839],["E",0.0220]]
    rospy.init_node("navegacion_node",anonymous=True)
    nav=navegacion(x,y,width,height,coordenadas,orientacion_inicial)


    #nav.show()
    rospy.spin()


if __name__ == '__main__':
    main()
