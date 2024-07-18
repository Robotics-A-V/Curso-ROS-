# FETCH ROBOT

A continuación vamos a revisar el funcionamiento del robot perteneciente a Fetch Robotics https://docs.fetchrobotics.com/

1. Descargar el archivo Fetch.zip [Fetch.zip ](https://drive.google.com/file/d/1BIxx2r0pQhecf6fCdnr5PlkOMnW-xIVL/view?usp=drive_link)
2. Copiarlo al src del work space
3. Abrir la terminal

con el comando rosdep garantizamos que todas las dependencias están instaladas.
```
rosdep install --from-paths src --ignore-src -r -y
```
4. Compilamos.

Empezar la simulacion

```
roslaunch fetch_gazebo simulation.launch
```

Revisamos todos los topicos que incluyen para el control del robot

```
rostopic list
```
```
rostopic info /base_controller/command
```
```
rosmsg info geometry_msgs/Twist
```

Movemos el robot
```
rostopic pub /base_controller/command geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r3
```

Creamos el  paquete para el control  y compilamos

```
catkin_create_pkg package_fetch roscpp rospy sensor_msgs control_msgs geometry_msgs trajectory_msgs actionlib actionlib_msgs
```

