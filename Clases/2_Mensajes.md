# Creacion de un espacio de trabajo
* Creamos la carpeta de un espacio de trabajo con el siguiente comando:

```
mkdir [Nombre_del_espacio_de_trabajo]
```
* Creamos una subcarpeta llamada src dentro del espacio de trabajo

```
cd [Nombre_del_espacio_de_trabajo]
mkdir src
```
* Compilamos dentro del espacio de trabajo con:

```
catkin_make
```
* Agregamos el nombre del espacio de trabajo creado en la parte final del archivo .bashrc con el sigunte comando:

```
source ~/[Nombre_del_espacio_de_trabajo]/devel/setup.bash
```
* El archivo .bashrc se lo encuentra en la carpeta personal del usuario, activando la visibilidad de los archivos ocultos.
# Creacion de un paquete
* Entramos a la carpeta src de espacio de trabajo creado, mediante:

```
cd src
```
* Creamos un paquete usando el siguiente comando:

```
catkin_create_pkg [Nombre_del_paquete] [Nombres_de_las_dependencias_del_paquete]
```
Entre las dependencias mas comunes estan:
1. roscpp: Es la biblioteca principal de C++ utilizada para desarrollar nodos y aplicaciones en ROS.

2. rospy: Es la biblioteca principal de Python utilizada para desarrollar nodos y aplicaciones en ROS.

3. std_msgs: Es un conjunto de mensajes estándar que proporciona tipos de datos comunes, como cadenas, números enteros y booleanos.

4. sensor_msgs: Proporciona mensajes para datos de sensores, como imágenes, datos de láser, datos de odometría, etc.

5. geometry_msgs: Contiene mensajes relacionados con geometría y transformaciones, como mensajes de posición, orientación, velocidad, etc.

6. visualization_msgs: Proporciona mensajes para visualización, como marcadores, representaciones de objetos en RViz, etc.

7. tf: Es una biblioteca que proporciona herramientas para trabajar con transformaciones en ROS. Se utiliza para administrar y publicar transformaciones entre diferentes sistemas de coordenadas.

8. rviz: Es una herramienta de visualización 3D en ROS que se utiliza para mostrar visualmente datos del robot, como sensores, modelos, etc.


* Regresamos a la carpeta del espacio de trabajo con:

```
cd ..
```
Compilamos con:
```
catkin_make
```
# Creacion de nodos en ROS

* Entramos a la carpeta src dentro del paquete anteriormente creado, usando los comandos:

```
cd src
cd [Nombre_del_paquete]
cd src
```
* Creamos un nuevo archivo .py usando el comando

```
gedit [Nombre_del_archivo.py]
```
* Le damos caracteristicas de ejecucion mediante el comando

```
chmod +x [Nombre_del_archivo.py]
```
* La estructura basica de un nodo publicador posee el siguiente codigo escrito en python, dicho codigo publica un "Hola mundo" en un topic de ROS

```
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('Topic_mensaje', String , queue_size=1)
	rospy.init_node('Nodo_publicador',anonymous=True)
	rate = rospy.Rate(1) # 1 Hz
	while not rospy.is_shutdown():
		pub.publish("Hola mundo")
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
```
* Ejemplo de entrada de dos numeros y publicacion de su suma y resta
```
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def talker():
	pub = rospy.Publisher('topic_suma', Int16 , queue_size=1)
	pub2 = rospy.Publisher('topic_resta', Int16 , queue_size=1)
	rospy.init_node('Nodo_operaciones',anonymous=True)
	rate = rospy.Rate(1) # 1 Hz
	while not rospy.is_shutdown():
		mensaje1 = int(input("Ingrese un numero: "))
		mensaje2 = int(input("Ingrese otro numero: "))
		pub.publish(mensaje1+mensaje2)
		pub2.publish(mensaje1-mensaje2)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
```
* La estructura basica de un nodo suscriptor posee el siguiente codigo escrito en python, dicho codigo se suscribe a un topic y muestra por consola el mensaje recibido.
```
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(dato):
	print("El mensaje recibido es: ")
	print(dato.data)

def funcion_principal():
	rospy.Subscriber("Topic_mensaje" , String , callback)
	rospy.init_node('Nodo_suscriptor',anonymous=True)
	rospy.spin()

if __name__=='__main__':
	try:
		funcion_principal()
	except rospy.ROSInterruptException:
		pass
```
# Creacion de mensajes propios en ROS
* Dentro de la carpeta src del espacio de trabajo creamos un nuevo paquete, el cual contendra los mensajes que generemos, la creacion de dicho paquete se realiza con el comando:

```
catkin_create_pkg [Nombre_del_paquete] [Nombres_de_las_dependencias_del_paquete]
```
* Se recomienda agregar las dependencias basicas de roscpp, rospy y std_msgs
* Regresamos a la carpeta del espacio de trabajo con:

```
cd ..
```
* Y compilamos con:

```
catkin_make
```
* Nos ubicamos dentro del paquete creado con:

```
cd [Nombre_del_nuevo_paquete]
```
* y creamos una carpeta llamda msg mediante el comando

```
mkdir msg
```
* nos ubicamos dentro de dicha carpeta con:

```
cd msg
```
* y creamos un nuevo archivo con extension .msg para ello escribimos:

```
gedit [Nombre_del_archivo_de_mensajes.msg]
```
* Dentro del nuevo archivo podemos escribir los atributos que tendra nuestro mensaje propio; se utiliza un lenguaje de descripción de mensajes llamado "ROS Message Description Language" o "ROS msg" para definir los mensajes en los archivos .msg, por ejemplo:

```
string Palabra1
string Palabra2
```
* Regresamos al paquete de mensajes con

```
cd ..
```
* y editamos el archivo CMakeList.txt mediante:

```
gedit CMakeLists.txt
```
* Para configurar correctamente este archivo se descomentarán las siguientes funciones y se agregarán los paquetes message_generation y message_runtime como se muestra a continuación:
1. Dentro de la sentencia *find_package* la cual se encuentra aproximadamente en la linea 10 del archivo, agregar el paquete *message_generation*, debe quedar de la siguiente manera:

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```
2. Descomentar la sentencia *add_message_files*  la cual se encuentra aproximadamente en la linea 51 y agregar el nombre del archivo de mensajes que hayamos creado, debe quedar de la siguiente manera:

```
add_message_files(
   FILES
#   Message1.msg
#   Message2.msg
   [Archivo_de_mensajes_creado.msg]
)
```
3. Descomentar la sentencia *generate_messages* la cual se encuentra aproximadamente en la linea 73, debe quedar de la siguiente manera:

```
generate_messages(
   DEPENDENCIES
   std_msgs
)
```
4. Agregar el paquete *message_runtime* dentro de la sentencia *catkin_package* la cual inicial en la linea 107 aproximadamente, el paquete debe ser agregado en la linea de "CATKIN_DEPENDS", debe quedar de la siguiente manera:

```
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Mensajes_propios
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```
* Guardamos el archivo CMakeList.txt y editamos el archivo package.xml mediante:

```
gedit package.xml
```
* Agregamos los paquetes message_generation y message_runtime en el bloque de programacion que se encuentra entre las lineas 50 y 63 aproximadamente, dicho bloque debe quedar de la siguiente manera:

```
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
```
* Guardamos el archivo package.xml
* Creamos un nuevo paquete en cuyas dependencias se debe incluir el paquete de mensajes que acabamos de crear
* Los nodos del nuevo pueden pueden usar los mensajes generados
* El siguiente ejemplo de nodo publicador importa un paquete generado llamado *mensajes_propios.msg* del cual importa tipo de mensaje llamado *mi_mensaje*, el mismo consta de dos strings como se va continuacion
```
#!/usr/bin/env python3

import rospy
from Mensajes_propios.msg import mi_mensaje

def talker():
	pub = rospy.Publisher('Topic_mensaje',  mi_mensaje , queue_size=1)
	rospy.init_node('Nodo_publicador',anonymous=True)
	rate = rospy.Rate(1) # 1 Hz
	while not rospy.is_shutdown():
		pub.publish("Hola mundo","Como estas?")
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
```
* Ejemplo del suscriptor con mensajes propios
```
#!/usr/bin/env python3

import rospy
from mensajes_propios.msg import mi_mensaje

def callback(dato):
	print("La primera palabra es: ")
	print(dato.Palabra1)
	print("La segunda palabra es: ")
	print(dato.Palabra2)

def funcion_principal():
	rospy.Subscriber("Topic_mensaje" , mi_mensaje, callback)
	rospy.init_node('Nodo_suscriptor',anonymous=True)
	rospy.spin()

if __name__=='__main__':
	try:
		funcion_principal()
	except rospy.ROSInterruptException:
		pass
```
