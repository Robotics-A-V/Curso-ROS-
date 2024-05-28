
#############################   Configuracion inicial ######################################################3
https://github.com/xArm-Developer/xarm_ros

#clonar el repositorio dentro de la carpeta src / abro una terminal
```
git clone https://github.com/xArm-Developer/xarm_ros.git --recursive
```

#Actualizar paquetes 

```
sudo apt-get update
```

#Ingreso a la carpeta xarm_ros
```
cd xarm_ros
```

#Actualizo las dependencias de los paquetes
```
git pull
git submodule sync
git submodule update --init --remote
```

#Actualizo otros paquetes pendientes
```
rosdep update
```

```
rosdep check --from-paths . --ignore-src --rosdistro noetic
```
```
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
```
#De no funcionar el codigo utilizar
```
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-moveit
sudo apt-get install ros-melodic-moveit-ros-move-group ros-melodic-moveit-ros-planning ros-melodic-moveit-ros-visualization
sudo apt-get install ros-melodic-moveit-visual-tools ros-melodic-moveit-servo
sudo apt-get install ros-melodic-joy
```
# Regreso al espacio de trabajo abro un terminal y compilo 
```
catkin_make
```
# Revisión de comandos Rviz 
```
roslaunch xarm_description xarm7_rviz_display.launch
```
# Revision de comandos Gazebo
```
roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true] [add_gripper:=true] [add_vacuum_gripper:=true] 
```
############# Movimiento ###################################################

# Realizar simulación de movimientos unicamente rviz 
```
roslaunch xarm7_moveit_config demo.launch
```
# Realizar simulación de movimientos usando rviz y gazebo
```
roslaunch xarm_gazebo xarm7_beside_table.launch    # Terminal 1 
```
```
roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch #Terminal 2
```
# Realizar el movimiento en el robot original con moveit
```
roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=192.168.0.1 [velocity_control:=false] [report_type:=normal]
```

Simulador actual
```
roslaunch xarm_planner xarm_planner_rviz_sim.launch robot_dof:=6 robot_type:=lite  add_gripper:=false add_vacuum_gripper:=true
```
https://github.com/xArm-Developer/xarm_ros/blob/master/xarm_planner/ReadMe.md
```
rosservice call xarm_joint_plan 'target: [1.0, -0.5, 0.0, -0.3, 0.0, 0.0]'
```
