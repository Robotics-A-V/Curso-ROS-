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

* Estructura básica de los nodos

Utilizando la estructura del bucle **not rospy.is_shutdown()**
```
#!/usr/bin/env python3

import rospy

def main():
    # Inicializa el nodo con el nombre 'primer_nodo'
    rospy.init_node('primer_nodo', anonymous=True)
    # Define la frecuencia del bucle en 1 Hz
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        rospy.loginfo("Mi primer nodo en ROS - while not rospy.is_shutdown()")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```
Utilizando la estructura **rospy.spin**
```
#!/usr/bin/env python3

import rospy

def timer_callback(event):
    rospy.loginfo("Mi primer nodo en ROS - rospy.spin()")

def main():
    # Inicializa el nodo con el nombre 'segundo_nodo'
    rospy.init_node('segundo_nodo', anonymous=True)
    # Configura un temporizador que llama a 'timer_callback' cada segundo
    rospy.Timer(rospy.Duration(1), timer_callback)
    # Mantiene el nodo activo
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```
