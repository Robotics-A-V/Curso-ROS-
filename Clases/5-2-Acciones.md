# Creacción de Acciones en ROS

Las acciones en ROS proporcionan un mecanismo para llevar a cabo tareas más complejas y con un control más sofisticado. Una acción consta de un goal (objetivo), un result (resultado) y un feedback (retroalimentación).

El objetivo (goal) define la tarea que se debe realizar.

El resultado (result) contiene el estado final de la tarea

La retroalimentación (feedback) proporciona información intermedia sobre el progreso de la tarea.

Las acciones se definen en archivos .action y requieren un servidor de acción y uno o varios clientes de acción.

Creamos el paquete 
```
catkin_create_pkg pkg_action_a roscpp rospy std_msgs actionlib actionlib_msgs message_generation geometry_msgs 
```

Ingresamos a nuestro paquete creado y creamos una carpeta llamada action

```
mkdir action
```
y ejecutamos visual studio code

```
code .
```

# Trayectorias

trayectoria2.action

* Dentro de la carpeta action creamos un nuevo archivo con el nombre "trayectoria2.action"

* Agregamos la estructura del archivo
```
# Definimos el objetivo(goal)
geometry_msgs/Twist[] velocidades  # Especificamos la posición en Posición en angular
---
# Definimos el resultado(result)
int32 total_distancia_recorrida # Total de trayectoria recorrida
---
# Definimos la retroalimentación(feedback)
float32 percent_distancia_recorrida # Porcentaje completado
```

* moficamos el archivo CMakelist.list de la siguiente forma:

es necesario agregar el archivo .action

nota: cuando no se agregan desde el inicio las dependencias action es necesario agregar  los siguientes paquetes: actionlib, actionlib_msgs, message_generation, geometry_msgs

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib  #paquete agregado
  actionlib_msgs #paquete agregado
  message_generation #paquete agregado
  geometry_msgs #paquete agregado para el uso de un tipo de mensaje específico
)

add_action_files(
    FILES
#   Action1.action
#   Action2.action
    trayectoria2.action
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   geometry_msgs
 )
```
Nota: tambien es necesario remplazar la linea 110 aproximandemente por:

```
  CATKIN_DEPENDS actionlib_msgs actionlib
```

Nota: mientras que en el archivo package.xml es necesario agregar las siguientes dependencias 

```
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>message_generation</build_depend>
<build_export_depend>actionlib</build_export_depend>
<build_export_depend>actionlib_msgs</build_export_depend>
<exec_depend>actionlib</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
<build_depend>geometry_msgs</build_depend>
<build_export_depend>geometry_msgs</build_export_depend>
<exec_depend>geometry_msgs</exec_depend>
```

revisamos que esté funcionando usando el comando 
```
rosmsg list | grep trayectoria2
```

Dentro de la carpeta src agrego los siguientes programas 

trayectoria_servidor.py
```
#!/usr/bin/env python3
# encoding: utf-8


import rospy       #Importamos ropsy (interface de python-ROS)

import actionlib     #Importamos actionlib para hacer uso de acciones en ROS

from paquete_acciones.msg import trayectoria2Action,trayectoria2Result,trayectoria2Feedback
#Importamos los mensajes de nuestra acción

from geometry_msgs.msg import Twist

class DoActionServer:

    def __init__(self):
        self.server = actionlib.SimpleActionServer('control_de_trayectoria', trayectoria2Action, self.execute, False)  #Declaramos nuestra Acción Server con nombre do_wash_car
        self.server.start()
        print("Corriendo el  action para realizar el control de trayectoria")

    def execute(self, goal):

        feedback = trayectoria2Feedback()      #Declaramos una variable de tipo Feedback
        result = trayectoria2Result()          #Declaramos una variable de tipo Result
        rate = rospy.Rate(10)                 #Loop 10hz
        pub = rospy.Publisher("/turtle1/cmd_vel", Twist , queue_size=10)
        for punto in range(0,len(goal.velocidades)):
            pub.publish(goal.velocidades[punto])
            result.total_distancia_recorrida += 1
            feedback.percent_distancia_recorrida = (result.total_distancia_recorrida*100.0)/len(goal.velocidades)
            self.server.publish_feedback(feedback)                #Publicamos el feedback
            rate.sleep()

        self.server.set_succeeded(result)                         #Publicamos el resultado


if __name__ == '__main__':
    rospy.init_node('nodo_action_servidor_trayectoria')
    server = DoActionServer()                                      #Creamos una instancia de la Clase DoActionServer
    rospy.spin()
```
trayectoria_cliente.py
```
#!/usr/bin/env python3
# encoding: utf-8

import rospy                                                    #Importamos ropsy (interface de python-ROS)
import actionlib                                                #Importamos actionlib para hacer uso de acciones en ROS
from paquete_acciones.msg import trayectoria2Action, trayectoria2Goal   #Importamos los mensajes de nuestra acción
from geometry_msgs.msg import Twist

def feedback_cb(msg):                                           #Definimos una función feedback_cb

    print('Feedback received -> '+str(msg)+'%')                 #Imprimimos en pantalla el feebback que envía el Action Server


def call_server():                                                        #Definimos una función call_server

    client = actionlib.SimpleActionClient('control_de_trayectoria', trayectoria2Action) #Declaramos nuestra Acción Cliente con nombre do_wash_car

    client.wait_for_server()                                              #Si el Action Server no está disponible; esperamos

    goal = trayectoria2Goal ()                                                #Definimos nuestra variable de tipo Goal

    twist1 = Twist()
    twist1.linear.x = 1.0
    twist1.linear.y = 2.0
    twist1.angular.z = 0.0

    twist2 = Twist()
    twist2.linear.x = 3.0
    twist2.linear.y = 4.0
    twist2.angular.z = 0.0

    twist3 = Twist()
    twist3.linear.x = 5.0
    twist3.linear.y = 6.0
    twist3.angular.z = 0.0

    twist4 = Twist()
    twist4.linear.x = 6.0
    twist4.linear.y = 7.0
    twist4.angular.z = 0.0

    twist5 = Twist()
    twist5.linear.x = 6.0
    twist5.linear.y = 8.0
    twist5.angular.z = 5.0

    goal.velocidades = [twist1, twist2, twist3, twist4, twist5]                                           #Definimos el número de Automóviles

    client.send_goal(goal, feedback_cb=feedback_cb)                       #Enviamos nuestro objetivo, y pasamos una función de Feedback

    client.wait_for_result()                                              #Esperamos el resultado hasta que el Action Server procese todo

    result = client.get_result()                                          #Obtenemos el resultado

    return result                                                         #Retornamos el resultado


if __name__ == '__main__':

    try:
        rospy.init_node('nodo_action_cliente_trayectoria')                            #Definimos el nombre de nuestro nodo

        result = call_server()                                           #Llamamos a nuestra función call_server()

        print("The result is: ", result)                                 #Imprimimos en pantalla el resultado

    except rospy.ROSInterruptException :                                 #Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass
```
