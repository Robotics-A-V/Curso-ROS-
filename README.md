## Introducción a ROS y Coppeliasim

Este repositorio contiene una serie de instrucciones para abordar la programacion de robots con ROS y coppealiasim usando python.

# Contenido
* [0. Prerrequisitos ](#0.-Prerrequisitos )
    * [0.1 Instalación de Ubuntu](#0.1-Instalación-de-Ubuntu)
    * [0.2 Instalación de ROS](#0.2-Instalación-de-ROS)
* [1. Introducción](#1-Introducción)
* [2. GitHub y ROS  ](#2-GitHub-y-ROS )
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
# 0.1 Instalación de Ubuntu

Para el funcionamiento de ROS es necesario instalar Ubuntu, para el actual repositorio se usará Ubuntu 20 y ROS NOETIC.

Imagen ISO (ubuntu-20.04.6-desktop-amd64.iso): [Descarga](https://releases.ubuntu.com/focal/)

Opción Máquina Virtual: [guia by Sarah Meyer­Waldo](https://drive.google.com/file/d/16WalqxyXt-MuOT5KX8L0hKnSa88WGsNh/view?usp=share_link)

Opcion Partición del disco (Cambiar por la Iso 20.04): [Tutorial](https://www.youtube.com/watch?v=_d6oT7rEoGc)

Verificación de instalación de Python

Ejecuto en la terminal
```
python3 --version
```

# 0.2 Instalación de ROS

Enlace Oficial ROS Noetic: [link](https://wiki.ros.org/noetic/Installation/Ubuntu)

Instalación de ROS Noetic en Ubuntu 20.04 

A continuación, ejecutar todos los comandos en el terminal:

[***1-Instalacion ROS***](./Doc/0_Instalacion.md)
# 1. Introducción
Conceptos generales sobre el uso de ROS [Resumen](./Doc/1_ROS.pdf)

[Wiki-ROS-Conceptos](https://wiki.ros.org/es/ROS/Conceptos)

[Wiki-ROS-Tutoriales](https://wiki.ros.org/ROS/Tutorials)
# 2. GitHub y ROS 

GitHub y ROS (Robot Operating System) están estrechamente relacionados en el desarrollo de software robótico:

1. **Alojamiento del Código**

Los desarrolladores de ROS usan GitHub para almacenar y gestionar el código fuente de sus paquetes de ROS en repositorios públicos o privados.

2. **Gestión de Dependencias** 

GitHub permite organizar proyectos de ROS en múltiples repositorios y submódulos, facilitando la gestión de dependencias entre paquetes.

3. **Colaboración**

GitHub proporciona herramientas como pull requests e issues, esenciales para la colaboración en proyectos de ROS, permitiendo a los desarrolladores contribuir y mejorar el software colectivamente.

4. **Versionado y Lanzamientos**

Los proyectos de ROS utilizan GitHub para etiquetar versiones y crear lanzamientos, asegurando que los usuarios puedan acceder a versiones específicas y estables del software.

5. **Integración Continua**

Con servicios de CI como GitHub Actions, los desarrolladores de ROS pueden automatizar pruebas y compilaciones, garantizando la calidad del código antes de su integración.

6. **Documentación**

GitHub facilita la publicación de documentación y guías de uso directamente en los repositorios, complementando la wiki oficial de ROS.

[Tutorial recomendado](https://www.youtube.com/watch?v=mBYSUUnMt9M&t=14612s)

[***2-Comandos Básicos***](./Doc/2_comandos_github.md)

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
