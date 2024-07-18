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

## Robot Movil

```
#!/usr/bin/env python3                         
# encoding: utf-8

import rospy                                  
from geometry_msgs.msg import Twist           #Importamos el tipo de mensaje Twist                                                          


def nodo():                                   #Definimos una función nodo                                   
    
    rospy.init_node('nodo_movimiento_base')         #Inicializamos nuestro nodo y le asignamos un nombre = nodo_movimiento_base
           
                                                # Name Topic   |tipo de mensaje|límite de 10 mensajes en cola
    velocity_publisher = rospy.Publisher('/base_controller/command',Twist, queue_size=10)

    # creamos un arreglo de mensajes tipo Twist (Para generar una trayectoria)
    vel_robot = []
    vel_referencia_x = [0.1,0.3,0.5,0.8]
    vel_referencia_y = [0.1,0.3,0.5,0.8]
    vel_referencia_w = [0.0,0.0,0.5,0.0]    
    for i in range(len(vel_referencia_x)):
        vel_msg = Twist()
        vel_msg.linear.x = vel_referencia_x[i]   #Asignamos una velocidad lineal a la componenete en X
        vel_msg.linear.y = vel_referencia_y[i]    #Asignamos una velocidad lineal a la componenete en Y
        vel_msg.angular.z = vel_referencia_w[i]   #Asignamos una velocidad angular en Z
        vel_robot.append(vel_msg)

    rate = rospy.Rate(10)     #Crea un objeto Rate a 10hz
    i = 0
    while not rospy.is_shutdown():              #Bucle While
        if(i > len(vel_referencia_x)-1):
             i = 0
        velocity_publisher.publish(vel_robot[i])     #publicamos nuestro mensaje vel_robot para mover la base del robot                                                
        i = i+1
        rate.sleep()                            #Loop 10 times per second           

if __name__ == '__main__':                                  
    try:
        nodo()                                 # Lamamos a la función nodo
    except rospy.ROSInterruptException :       # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
            pass
  ```

## Robot Articular



```
rostopic list | grep arm_controller
rostopic info /arm_controller/follow_joint_trajectory/goal
rosmsg info control_msgs/FollowJointTrajectoryActionGoal
```

https://docs.fetchrobotics.com/robot_hardware.html

```
rosparam get /arm_controller/follow_joint_trajectory/joints

rostopic pub /arm_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal
```

CODIGO

```
#!/usr/bin/env python3
# encoding: utf-8

import rospy      #Importamos ropsy (interface de python-ROS)
import actionlib  #Importamos actionlib para hacer uso de acciones en ROS

#Importamos los mensajes de la acción FollowJointTrajectory - Action y Goal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

#Importamos los mensajes de tipo JointTrajectory - JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#Importamos math para convertir Grados a Radianes en la generación de las trayectorias del Brazo del robot
import math

#Creamos una Lista de los nombres de las Articulaciones del Brazo del robot
arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]


'''
    trajectory_msgs/JointTrajectory.msg
        Header header
        string[] joint_names
        JointTrajectoryPoint[] points
    '''


    
# Creamos una trayectoria con 6 puntos

'''
    trajectory_msgs/JointTrajectoryPoint.msg

        Each trajectory point specifies either positions[, velocities[, accelerations]]
        or positions[, effort] for the trajectory to be executed.
        All specified values are in the same order as the joint names in JointTrajectory.msg

        float64[] positions
        float64[] velocities
        float64[] accelerations
        float64[] effort
        duration time_from_start

'''

trayectoria_F = []
for i in range(6):
    puntos = JointTrajectoryPoint()
    puntos.positions = [0.0, 0.0, math.radians(i*10), 0.0, 0.0, 0.0, 0.0]
    puntos.velocities = [0.0]
    puntos.accelerations = [0.0]
    puntos.time_from_start = rospy.Duration(1.0)  #Asignamos la duración de 1seg para iniciar
    #Creamos la trayectoria que contiene un arreglo de puntos y las etiquetas de las juntas a moverse.
    trajectoria = JointTrajectory() #Declaramos una variable de tipo JointTrajectory.msg
    trajectoria.joint_names = arm_joint_names  #Asignamos la lista de los nombres de las articulaciones del Brazo del Robot
    trajectoria.points.append(puntos)
    trayectoria_F.append(trajectoria)


print(len(trajectoria.points))
if __name__ == "__main__":
    
    rospy.init_node("nodo_move_arm") #Definimos el nombre de nuestro nodo

    rospy.loginfo("Waiting for arm_controller...") #Imprimimos en pantalla

                #Creamos una Acción Cliente|nombre de la acción :follow_joint_trajectory|especificación de la acción
    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    
    arm_client.wait_for_server() #Si el Acción Server no está disponible, esperamos

    rospy.loginfo("...connected.") #Imprimimos en pantalla
    
    arm_goal = FollowJointTrajectoryGoal() #Creamos una variable de tipo Goal
    arm_goal.trajectory = trayectoria_F[0] #Asignamos nuestra trayectoria hacia la trayectoria de nuestro objetivo(goal)
    arm_goal.goal_time_tolerance = rospy.Duration(0.0) #Asignamos la duración de 0seg como tiempo de tolerancia de nuestro objetivo
    rospy.loginfo("Setting positions...") #Imprimimos en pantalla
    arm_client.send_goal(arm_goal) #Enviamos nuestro objetivo hacia el Acción Server de nuestro robot Fetc
    arm_client.wait_for_result(rospy.Duration(6.0)) #Esperamos 6 segundos para obtener los resultados
    rospy.loginfo("...done") #Si todo se ha realizado correctamente imprimimos en pantalla el mensaje done(hecho)
    rospy.sleep(5)

    for i in range(5):
        arm_goal = FollowJointTrajectoryGoal() #Creamos una variable de tipo Goal
        arm_goal.trajectory = trayectoria_F[i] #Asignamos nuestra trayectoria hacia la trayectoria de nuestro objetivo(goal)
        arm_goal.goal_time_tolerance = rospy.Duration(0.0) #Asignamos la duración de 0seg como tiempo de tolerancia de nuestro objetivo
        rospy.loginfo("Setting positions...") #Imprimimos en pantalla
        arm_client.send_goal(arm_goal) #Enviamos nuestro objetivo hacia el Acción Server de nuestro robot Fetc
        arm_client.wait_for_result(rospy.Duration(6.0)) #Esperamos 6 segundos para obtener los resultados
        rospy.loginfo("...done") #Si todo se ha realizado correctamente imprimimos en pantalla el mensaje done(hecho)
```

## Sensores adicionales

Ingresamos al paquete Fetch

Ejecutamos en la terminal

```
code .
```
ingresamos a la carpeta /fetch_gazebo/robots

abrimos el archivo fetch.gazebo.xacro

Buscamos los parametros relacionados a base_laser

modificamos los siguientes parametros:

Original:
```
  <gazebo reference="laser_link">
    <sensor type="ray" name="base_laser">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>15</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>662</samples>
            <resolution>1</resolution>
            <min_angle>-1.91986</min_angle>
            <max_angle>1.91986</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.05</min>
          <max>25.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <!-- Noise parameters based on spec for SICK TIM561 (10m version) -->
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_base_hokuyo_controller" filename="libgazebo_ros_laser.so">
        <topicName>/base_scan</topicName>
        <frameName>laser_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
```
Cambios


```
  <!-- SICK TIM561 (25m Range) -->
  <gazebo reference="laser_link">
    <sensor type="ray" name="base_laser">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>15</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>240</samples>
            <resolution>1</resolution>
            <min_angle>-1.91986</min_angle>
            <max_angle>1.91986</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.05</min>
          <max>1.5</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <!-- Noise parameters based on spec for SICK TIM561 (10m version) -->
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_base_hokuyo_controller" filename="libgazebo_ros_laser.so">
        <topicName>/base_scan</topicName>
        <frameName>laser_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
```


revisamos los topicos existentes

revisamos la informacion del topico base_scan


```
rostopic info /base_scan
```

revisamos la estructura del mensaje


```
rosmsg info sensor_msgs/LaserScan
```

las muestras del laser se almacenan en el tipo de dato float32[] ranges


#Ejecucion de un escenario
```
roslaunch fetch_gazebo playground.launch
```
para escuchar una sola vez
```
rostopic echo /base_scan -n1
```

Visualización de la camara

```
#!/usr/bin/env python3                         
# encoding: utf-8

import rospy                                  
from geometry_msgs.msg import Twist           #Importamos el tipo de mensaje Twist                                                          
import math

global wref, uref, tetha

def calcular():
    global wref, uref, tetha
    uref =  [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    wref =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              -1, -1, -1,-1, -1, -1, -1, -1, -1,-1, -1, -1, -1, -1, -1,-1, -1, -1, -1, -1, -1,-1, -1, -1]
    tetha = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def nodo():                                   #Definimos una función nodo                                   
    
    rospy.init_node('nodo_movimiento_base')         #Inicializamos nuestro nodo y le asignamos un nombre = nodo_movimiento_base
           
                                                # Name Topic   |tipo de mensaje|límite de 10 mensajes en cola
    velocity_publisher = rospy.Publisher('/base_controller/command',Twist, queue_size=10)

    # creamos un arreglo de mensajes tipo Twist (Para generar una trayectoria)
    vel_robot = []

    vel_referencia_x = []
    vel_referencia_y = []
    vel_referencia_w = []
    for i in range(len(uref)):
        x = uref[i]*math.cos(tetha[i])
        y = uref[i]*math.sin(tetha[i])
        w = wref[i]
        vel_referencia_x.append(x)
        vel_referencia_y.append(y)
        vel_referencia_w.append(w)

    print("Cantidad de datos", len(vel_referencia_x))
    for i in range(len(vel_referencia_x)):
        vel_msg = Twist()
        vel_msg.linear.x = vel_referencia_x[i]   #Asignamos una velocidad lineal a la componenete en X
        vel_msg.linear.y = vel_referencia_y[i]    #Asignamos una velocidad lineal a la componenete en Y
        vel_msg.angular.z = vel_referencia_w[i]   #Asignamos una velocidad angular en Z
        vel_robot.append(vel_msg)

    rate = rospy.Rate(10)     #Crea un objeto Rate a 10hz
    i = 0
    while not rospy.is_shutdown():              #Bucle While
        if(i > len(vel_referencia_x)-1):
             break
        print("Velocidad: ", i)
        velocity_publisher.publish(vel_robot[i])     #publicamos nuestro mensaje vel_robot para mover la base del robot                                                
        i = i+1
        rate.sleep()                            #Loop 10 times per second           

if __name__ == '__main__':                                  
    try:
        calcular()
        nodo()                                 # Lamamos a la función nodo
    except rospy.ROSInterruptException :       # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
            pass
```


