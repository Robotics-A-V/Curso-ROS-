## Introducción a ROS y Coppeliasim

Este repositorio contiene una serie de instrucciones para abordar la programacion de robots con ROS y coppealiasim usando python.

# Contenido
* [0. Prerrequisitos ](#0.-Prerrequisitos )
* [1. Introducción](#1-Introducción)
* [2. Uso Básico de Git ](#2-Uso-Básico-de-Git)
* [3. Paquetes y Nodos ](#3-Paquetes-y-Nodos)
* [4. Intercambio de mensajes en ROS](#4-Intercambio-de-mensajes-en-ROS)
    * [4.1 Estructura de mensajes] 
    * [4.2 Mensajes Propios]
* [5. Servicios y Acciones](#5.-Servicios-y-Acciones)
    * [5.1 Servicios]
    * [5.2 Acciones]
* [6. Comunicación con dispositivos](#6.-Comunicación-con-dispositivos)
    * [6.1 Serial - Arduino ID]
    * [6.2 Mqtt - ESP32]
* [7. Robot 3 GDL](#7.-Robot-3-GDL)
* [8. Rviz MoveiT](#8.-Rviz-Movei)
* [9. Coppeliasim](#9.-Coppeliasim)
  
# 0. Prerrequisitos
1. Instalación de Ubuntu.

Para el funcionamiento de ROS es necesario instalar Ubuntu, para el actual repositorio se usará Ubuntu 20 y ROS NOETIC.

Imagen ISO (ubuntu-20.04.6-desktop-amd64.iso): [Descarga](https://releases.ubuntu.com/focal/)

Opción Máquina Virtual: [guia by Sarah Meyer­Waldo](https://drive.google.com/file/d/16WalqxyXt-MuOT5KX8L0hKnSa88WGsNh/view?usp=share_link)

Opcion Partición del disco (Cambiar por la Iso 20.04): [Tutorial](https://www.youtube.com/watch?v=_d6oT7rEoGc)

2. Instalacion de ROS

Enlace Oficial ROS Noetic: [link](https://wiki.ros.org/noetic/Installation/Ubuntu)

Instalación de ROS Noetic en Ubuntu 20.04 

A continuación, ejecutar todos los comandos en el terminal:

Paso 1 - Configura tu lista de fuentes
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Paso 2 - Configura tus llaves
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
Paso 3 - Actualizacion de claves
```
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
```
```
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Paso 3 - Instalacion :
```
sudo apt update
sudo apt install ros-noetic-desktop-full
```
Paso 4 - Configuración del entorno:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Paso 5 - Dependencias para construir paquetes :
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
Paso 6 - Inicializacion de  rosdep :
```
sudo apt install python3-rosdep
```
Paso 7 - Inicio de rosdep
```
sudo rosdep init
rosdep update
```
Comprobación:
```
roscore 
```
![xArmFrames](./doc/0_comprobacion_instalacion.png)   
# 1. Introducción
# 2. Uso Básico de Git
# 3. Paquetes y Nodos
# 4. Intercambio de mensajes en ROS
# 5. Servicios y Acciones
# 6. Comunicación con dispositivos
# 7. Robot 3 GDL
# 8. Rviz MoveiT
Instalar las dependencias
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing> 
   
   ros_control: <http://wiki.ros.org/ros_control> 
   
   moveit_core: <https://moveit.ros.org/install/>  
# 9. Coppeliasim
