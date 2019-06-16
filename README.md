#   SIMULACION DE NAVEGACION UTLIZANDO CAMPOS POTENCIALES
##  Robotica Movil

_Luis Gerardo Bernabe Gomez_

## Requisitos:
- Python
- ROS
- Kobuki. (más informacion [aqui](http://wiki.ros.org/kobuki/Tutorials/Simulated%20Kobuki%20navigation%20in%20perfect%20world)  )

## Instalacion:
Donde se encuentra la carpeta _devel_ y _src_ se ejecutan los siguientes comandos. 

```
$ catkin make
$ source devel/setup.bash
```
__Nota:__ En cada terminal que se abra se tiene que realizar el procedimiento anterior. 

Consiste en tres etapas: 

1. Correr RosCore y todo el entorno grafico simulando un salon de clases
2. Levantar el nodo que crea el publicador para dar probabilidad de ocupacion en el grid.
3. Ejecutar el algoritmo de navegacion.  

__A continuación el procedimiento para ejecutar cada uno de ellos__ 

### 1. Correr RosCore y entorno grafico

Ejecutamos el script _salon.launch_ el cual primero levantara el roscore para despues levantar 
todos los nodos que simulan las sillas, mesas y paredes definidas en el script. 
```
$ roslaunch mesa_marker salon.launch
```

En otra terminal 

### 2. Publicador del Occupancy Grid

El archivo _src/piso_salon/lugares_ocupados.csv_ contiene la informacion de la probabilidad de ocupacion del grid en cada coordenada. 

En otra terminal ejecutamos el script para crear el publicador que mandara la informacion en rviz.

```
$ rosrun piso_salon piso.py 
```

### 3. Ejecutar el algoritmo de navegacion.

Ejecutamos el algoritmo de navegacion. 

```
$ rosrun navegacion navegacion.py 
```


 



