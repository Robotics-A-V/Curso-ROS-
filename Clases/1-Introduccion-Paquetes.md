# Paquete: Turtlesim
* Iniciamos el entorno de ros ejecutando el siguiente comando en una nueva terminal

```
roscore
```
* En una nueva terminal ejecutamos el siguiente comando para lanzar el simulador de la tortuga

```
rosrun turtlesim turtlesim_node
```
* Podemos controlar la tortuga ejecutando el siguiente nodo

```
rosrun turtlesim turtle_teleop_key
```
* Podemos observar la posicion y orientacion de la tortuga mediante:

```
rostopic echo /turtle1/pose
```
* Podemos ver las conexiones entre nodos y topic mediante:

```
rosrun rpt_graph rqt_graph
```
En el contexto de ROS (Robot Operating System), "rosout" se refiere a un nodo especial llamado "rosout" que está presente en cada ejecución de ROS. El nodo "rosout" es responsable de la gestión y el registro de los mensajes de salida (logs) generados por otros nodos en el sistema.

Cuando los nodos de ROS envían mensajes de salida o registran eventos, esos mensajes se enrutan al nodo "rosout". Luego, el nodo "rosout" los procesa y los almacena en un archivo de registro llamado "rosout.log". Estos mensajes de salida pueden contener información de depuración, advertencias, errores u otra información relevante generada por los nodos de ROS durante la ejecución.

En resumen, "rosout" es un nodo en ROS encargado de gestionar y registrar los mensajes de salida de los demás nodos del sistema.

# Paquete: find_object_2d
* Instalacion del paquete

```
sudo apt install ros-noetic-find-object-2d
```

* Clonamos el codigo fuente en la carpeta find_object_2d que esta dentro de la carpeta src

```
git clone https://github.com/introlab/find-object.git src/find_object_2d
```
* Compilamos escribiendo el comando:

```
catkin_make
```

* Instalamos el paquete de dependencia usb-cam

```
sudo apt install ros-noetic-usb-cam
```
* Proceso para ejecutar el nodo para detectar objetos: iniciamos el controlador ROS usb-cam

```
roscore
roslaunch usb_cam usb_cam-test.launch
```
* El comando iniciará el controlador ROS de la cámara web USB, podemos usar el comando rostopic list para enumerar los temas en este controlador. Se utilizará la imagen original de la cámara y la imagen se publicará en el tema / usb_cam / image_raw. El siguiente paso es ejecutar el nodo detector de objetos con el siguiente comando:

```
rosrun find_object_2d find_object_2d image:=/usb_cam/image_raw
```

* Pasos para la detección de objetos:
1. Hacer clic derecho en el panel izquierdo (Objetos) de la ventana, aparecerá
la opción “Agregar objetos de la escena”, seleccionar esta opción, aparecerá un cuadro de diálogo “Agregar objeto”.
2. Después de apuntar el objeto a la cámara, presione el botón “Tomar foto” para obtener una instantánea del objeto.
3. La siguiente ventana se usa para marcar objetos de la instantánea actual. Primero usamos el puntero del mouse para marcar el objeto de selección de marco, luego haga clic en el botón "Siguiente" para recortar el objeto y luego continúe con el siguiente paso.
4. Después de recortar el objeto, se mostrará el número total de descriptores de características del objeto, puede hacer clic en el botón "Finalizar" para agregar esta plantilla de objeto para su detección.

* Se puede usar el siguiente comando para consultar la posicion del objeto

```
rosrun find_object_2d print_objects_detected
```
