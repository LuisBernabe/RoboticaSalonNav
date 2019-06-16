#!/usr/bin/env python


#Clase que utiliza occupancy grid para mostrar que lugares en el salon estan ocupados
#width=ancho= 720 (24)
#height=largo=900 (30 azulejos)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
import rospy
import parser_csv

class Piso:
    def __init__(self):
        self.piso_publicador=rospy.Publisher("/piso_publicador",OccupancyGrid,queue_size=10)

        width=24    #X
        height=30 #Y
        origen_x=-(width*0.3 )#/2)
        origen_y=-(height*0.3)# /2)
        origen=Pose()
        map_msg=OccupancyGrid()
        map_msg.header.frame_id="odom"
        map_msg.info.width=width
        map_msg.info.height=height
        map_msg.info.resolution=0.30 #30 cm por celda
        #origen.position.x=origen_x
        #origen.position.y=origen_y
        map_msg.info.origin=origen

        self.grid_numpy=np.zeros((height,width),dtype=int)

        for w in range(width):
            for h in range(height):
                self.grid_numpy[h][w]=40
                #origen.position.z=0
        self.llena_ocupados()
        grid=self.grid_numpy.tolist()
        #self.llena_ocupados()
        map_msg.data=sum(grid,[]) #El grid ya esta en ceros
        #self.piso_publicador.publish(map_msg)

        #print("Listo :D ")
        while not rospy.is_shutdown():
            self.piso_publicador.publish(map_msg)
            print("Listo :D ")

    """
        Logra convertir los datos que se encuentran en el archivo cvs e ingresar su valor en 
        el occupancy grid. 
    """
    def llena_ocupados(self): ## TODO
        parser=parser_csv.ParserCsv()
        #path="home/berna"
        lista_ocupados=parser.parse_file("lugares_ocupados.csv")
        for item in lista_ocupados:
            x=int(item.get('x'))
            y=int(item.get('y'))
            probaOcupado=int(item.get('probaOcupado'))
            self.grid_numpy[y][x]=probaOcupado
        #print(lista_ocupados)


if __name__ == '__main__':
    rospy.init_node("piso_grid", anonymous=True)
    piso=Piso()
    rospy.spin()
