# Instalación del complemento
Pasos para la instalación del complemento URDF en SolidWorks
1. Podemos encontrar el instalador del complemento en el siguiente enlace:
[Enlace de descarga del complemento de SolidWorks](http://wiki.ros.org/sw_urdf_exporter)
2. Damos click en el botón "Download Installer" y nos dirigirá a un repositorio en github.
3. Buscamos el complemento adecuado para la versión de SolidWorks que tengamos instalada y descargamos su corresponiente archivo *sw2urdfSetup.exe*; se recomienda la versión 2019.
4. Ejecutamos el archivo *sw2urdfSetup.exe* como administrador, damos *next* a todoas las ventanas que lo solicite y finalizamos al instalación. Nota: SolidWorks debe estar cerrado para realizar el proceso
5. Abrimos SolidWorks, nos dirigimos a la opción de complementos y habilitamos el complemento *SW2URDF*

# Exportación de modelos URDF desde SolidWorks
1. Una vez creado el ensamble abrimos el complemento el cual se encuentra en la parte final de la lista de herramientas. El Complemento se encuentra dentro de la opción *File*.
2. Configuramos los eslabones como padre e hijos respectivamente asigando nombres a los eslabones y a las juntas; y selecionando el tipo de junta correspondiente.
3. Damos click a la opción *Preview and Export*.
4. Se abrirá una nueva ventana donde se revisará que todas las configuraciones estén correctamente establecidas.
5. De ser necesario se puede cambiar la posición y orientación de los diferentes sistemas de coordenadas dando click en el icono del sistema de coordenadas en el arbol de operaciones y dando click en *Editar operación*.
6. Dar click en *Next* y de ser necesario editar las caracteristicas físicas de los eslabones.
7. Dar click en *ExportURDF and Meshes* y seleccionamos una ubicación para guardar el modelo.
8. Se generarán varias carpetas, si queremos comprobar el estado final del modelo puedemos arrastrar toda la carpeta al siguiente visualizador:
[Enlace del visualizador de modelos URDF](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html)

# Creacion del espacio de trabajo y paquetes
* Creamos un nuevo espacio de trabajo y un nuevo paquete
* Se recomienda el uso de las siguientes dependencias para el correcto funcionamiento del modelo:

```
catkin_create_pkg [Nombre_del_paquete] std_msgs rospy roscpp urdf xacro move_base_msgs nav_msgs robot_pose_ekf robot_state_publisher joint_state_publisher tf
```
Dentro del paquete se agregaran las carpetas:
* config
* launch
* meshes
* textures
* urdf

Dichas carpetas fueron generadas en al momento de exportar el modelo URDF desde el ensamble de SolidWorks

* Dentro de la carpeta *launch* se creara un nuevo archivo con la extencion *.launch*, se usara el siguiente comando:

```
gedit [nombre_del_archivo.launch]
```

La estructura basica de un archivo launch para ejecutar el simulador de turtlesim y el nodo de teleoperacion es la siguiente:
```
<launch>
  <!-- Lanza el simulador TurtleSim -->
  <node name="Nodo_simulador" pkg="turtlesim" type="turtlesim_node"/>

  <!-- Lanza el nodo de teleoperación -->
  <node name="Nodo_teleoperacion" pkg="turtlesim" type="turtle_teleop_key"/>
</launch>
```

La estructura basica de un archivo launch para ejecutar el modelo URDF de un robot de dos grados de libertad es la siguiente:
```
<launch>

   <!-- Robot Scara -->
   <arg name="model" default="Robot_Scara.urdf"/>

   <!-- Establece estos parametros en el servidor de parametros -->
   <param name="robot_description" textfile="$(find paquete_rviz)/urdf/$(arg model)" />

   <!-- El siguiente paquete publica el estado del modelo URDF -->
   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

   <!-- El siguiente paquete publica la transformacion entre marcos de referencia del entorno RVIZ y el eslabon base -->
   <node name="Transformacion_base" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0 map base 100" />

   <!-- El siguiente paquete publica la transormacion entre marcos de referencia del eslabon base y el eslabon 1 -->
   <node name="Transformacion_eslabon1" pkg="tf" type="static_transform_publisher" args="0 0.050 0 0 0 0 base eslabon1 100" />

   <!-- El siguiente paquete publica la transormacion entre marcos de referencia del eslabon 1 y el eslabon 2 -->
   <node name="Transformacion_eslabon2" pkg="tf" type="static_transform_publisher" args="0.100 0 0.040 0 0 0 eslabon1 eslabon2 100" />

   <!-- El siguiente paquete publica la transormacion entre marcos de referencia del eslabon 2 y el eslabon 3 -->
   <node name="Transformacion_eslabon3" pkg="tf" type="static_transform_publisher" args="0.070 0 0.040 0 0 0 eslabon2 eslabon3 100" />

</launch>
```
