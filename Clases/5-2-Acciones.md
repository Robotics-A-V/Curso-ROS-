# Creacción de Acciones en ROS

Las acciones en ROS proporcionan un mecanismo para llevar a cabo tareas más complejas y con un control más sofisticado. Una acción consta de un goal (objetivo), un result (resultado) y un feedback (retroalimentación).

El objetivo (goal) define la tarea que se debe realizar.

El resultado (result) contiene el estado final de la tarea

La retroalimentación (feedback) proporciona información intermedia sobre el progreso de la tarea.

Las acciones se definen en archivos .action y requieren un servidor de acción y uno o varios clientes de acción.

Creamos el paquete 
```
catkin_create_pkg paquete_acciones_a roscpp rospy std_msgs actionlib actionlib_msgs message_generation
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

nota: los paquetes que contiene action necesitan de los paquetes: actionlib, actionlib_msgs, message_generation, geometry_msgs
```

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
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

en el archivo package.xml
```
 <build_depend>geometry_msgs</build_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <exec_depend>geometry_msgs</exec_depend>
```
