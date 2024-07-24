# Cinemática Inversa

El objetivo de esta escena es, obtener los comandos básicos para poder realizar la cinemática inversa de cualquier tipo de robot articular.

Se plantea el uso de 2 metodos de resolución para la cinemática inversa:
1. Metodo de la seudoinversa.
2. Método de minimos cuadrados.

La escena generada os permitirá mover la posición y orientación del efector del robot elegido, en este caso el efector es un gripper.

# Cinemática Inversa y ejecicio de PICK and PLACE 

Utilizando la herramienta de cinemática inversa se implementa el ejercicio de PICK and PLACE.

En la escena se impementa la siguiente rutina:

1. Se establece una posición de HOME que facilite el movimiento del robot hacia los siguientes puntos. Además, se realiza la apertura del gripper.

2. La banda transportada mueve los objetos (cubo rojo y verde), y se detiene una vez que son detectados por sensor de proximidad.

3. El robot se dirige a la posición seteada para recoger los cubos, cierra el gripper y se mueve hacia la segunda posición seteada donde colocará el cubo.

4. El robot repite los puntos 1,2 y 3, y va colocando los cubos uno encima de otro.


# Comunicación ROS y teleop_twist_keyboard

A continuación, esta escena implementa el uso de la comunicación entre ROS y Coppeliasim. El objetivo de la misma es manejar el roboto presentado en la simulación a través del paquete teleop_twist_keyboard.

Para ejecutar la escena es necesario arrancar el nodo master de ROS a través del comando:

ROSCORE
