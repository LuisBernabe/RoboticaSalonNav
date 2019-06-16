#   SIMULACION DE NAVEGACION UTLIZANDO CAMPOS POTENCIALES
##  Robotica Movil

_Luis Gerardo Bernabe Gomez_

## Requisitos:
- Python
- ROS

## Instalacion:
Donde se encuentra la carpeta _devel_ y _src_ se ejecutan los siguientes comandos. 

```
$ catkin make
$ source devel/setup.bash
```

Consiste en tres etapas: 

1. Correr RosCore y todo el entorno grafico simulando un salon de clases
2. Levantar el nodo que crea el publicador para dar probabilidad de ocupacion en el grid.
3. Ejecutar el algoritmo de navegacion.  

__A continuación el procedimiento para ejecutar cada uno de ellos__ 

### 1. Correr RosCore

Ejecutamos el script _salon.launch_ el cual primero levantara el roscore para despues levantar 
todos los nodos que simulan las sillas, mesas y paredes definidas en el script. 
```
$ roslaunch mesa_marker salon.launch

```

### 2. Publicador del Occupancy Grid

