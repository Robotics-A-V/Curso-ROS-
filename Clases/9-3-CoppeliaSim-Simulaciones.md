# Cinemática Inversa

El objetivo de esta escena es, obtener los comandos básicos para poder realizar la cinemática inversa de cualquier tipo de robot articular.

Se plantea el uso de 2 metodos de resolución para la cinemática inversa:
1. Metodo de la seudoinversa.
2. Método de minimos cuadrados.

La escena generada os permitirá mover la posición y orientación del efector del robot elegido, en este caso el efector es un gripper.

[Escena](./ScenaTutorial.ttt)

# Cinemática Inversa y ejecicio de PICK and PLACE 

Utilizando la herramienta de cinemática inversa se implementa el ejercicio de PICK and PLACE.

En la escena se impementa la siguiente rutina:

1. Se establece una posición de HOME que facilite el movimiento del robot hacia los siguientes puntos. Además, se realiza la apertura del gripper.

2. La banda transportada mueve los objetos (cubo rojo y verde), y se detiene una vez que son detectados por sensor de proximidad.

3. El robot se dirige a la posición seteada para recoger los cubos, cierra el gripper y se mueve hacia la segunda posición seteada donde colocará el cubo.

4. El robot repite los puntos 1,2 y 3, y va colocando los cubos uno encima de otro.

[Escena](./ScenaTutorial_2.ttt)
# Comunicación ROS y teleop_twist_keyboard

(Para ejectuar la escena es necesario arrancar el programa mediante el comando ./start_coppelia.sh )
A continuación, esta escena implementa el uso de la comunicación entre ROS y Coppeliasim. El objetivo de la misma es manejar el robot presentado en la simulación a través del paquete teleop_twist_keyboard.

Para ejecutar la escena es necesario arrancar el nodo master de ROS a través del comando:

```
roscore
```

en una terminal adicional, colocamos el siguiente comando 

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Ejecutamos la escena y podremos controlar el robot móvil usando las teclas de nuestro teclado, el código necesario para implementar la comunicación entre ROS y CoppeliaSin se encuentra detallado en el scritp del robot móvil.
[Escena](./controlwheelV3.ttt)


# ControlledViaROS 
 
Esta escena es proporsioda por CoppealiaSim y contiene los bloques de programación necesarias para realizar la publicación y suscripción de tópicos a través de ROS.

Por la parte de suscripción se recibe la información del movimiento que debe tener el robot diferencial, a través del seteo de las velocidades en la llanta izquierda y derecha usando los tópicos: 

--'/'..leftMotorTopicName--  (aquí se realiza la unión de 2 cadenas de string a través del comando ".."). y el tópico: 

--'/'..rightMotorTopicName-- respectivamente.

Por la parte de publicación se envían los datos obtenidos por el sensor de proximidad y el valor del tiempo de simulación a través de los tópicos: 
--"/"..sensorTopicName --
-- '/'..simulationTimeTopicName -- , respectivamente.

