# Comunicación Serial con ROS

Primero debemos realizar la instalación de audiono ID. [Tutorial](https://www.youtube.com/watch?reload=9&v=VWycQGTYAwU)
# Instalacion
* Se instalara rosserial mediante el siguiente comando

```
sudo apt-get install ros-noetic-rosserial-arduino
```
* Descargar el software de Arduino del sitio oficial, la descarga debe realizarse de acuerdo a la infraestructura del sistema operativo con el que se esta trabajando: [Link de descarga oficial de arduino para distribuciones Linux](https://docs.arduino.cc/software/ide-v1/tutorials/Linux)
* Se descomprimira el archivo que se acaba de descargar
* Se abrirá una nueva terminal, se cambiará el directorio para encontrarse en la carpeta de descargas y se ejecutará el siguiente comando para ejecutar el archivo de instalación:

```
sudo sh install.sh
```
* Una vez completado este proceso se podrá encontrar el IDE de Arduino en la lista de aplicaciones
* Se instalará el paquete de ros_lib en el IDE. Para ellos se irá a Sketch, Incluir biblioteca, Administrar biblioteca y se buscará rosserial. A fin de evitar errores se recomienda rosserial en su versión 0.7.9

* Se instalara el paquete rosserial en ROS mediante el siguiente comando:

```
sudo apt-get install ros-noetic-rosserial-arduino
```

* Se conectara la placa Arduino UNO al computador a través de un cable USB, se seleccionara la
placa dentro de la lista de puertos seriales disponibles, se compilara el código y se subira. En caso de existir errores sera necesaria la apertura de puertos seriales, para ello se
abrira una nueva terminal y se escribira el siguiente comando:

```
ls -l /dev/ttyACM*
```
* Es necesario añadir el usuario mediante el siguiente comando:

```
sudo usermod -a -G dialout [Nombre_de_usuario]
```
* Para ejecutar el nodo de rosserial primero se ejecutara *roscore* y en una nueva terminal se escribira el comando verificando el numero de puerto en el que se encuentra conectada nuestra tarjeta:

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
# Nodo publicador en Arduino
* Ejecutado el nodo de rosserial, se puede tener acceso a los topicos declarados en la tarjeta Arduino, la estructura basica de un programa en Arduino IDE para la lectura y publicacion de dos se;ales analogicas es el siguiente:

```
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nodo_publicador_arduino;

std_msgs::Float32 angulo1_msg;
std_msgs::Float32 angulo2_msg;
std_msgs::Float32 desplazamiento_msg;

ros::Publisher publicador_angulo1("Topic_angulo1_arduino", &angulo1_msg);
ros::Publisher publicador_angulo2("Topic_angulo2_arduino", &angulo2_msg);
ros::Publisher publicador_desplazamiento("Topic_desplazamiento_arduino", &desplazamiento_msg);

void setup()
{
  nodo_publicador_arduino.initNode();
  nodo_publicador_arduino.advertise(publicador_angulo1);
  nodo_publicador_arduino.advertise(publicador_angulo2);
  nodo_publicador_arduino.advertise(publicador_desplazamiento);
}

void loop(){
  angulo1_msg.data = map(analogRead(A0) , 0 , 1023 , 0 , 180);
  angulo2_msg.data = map(analogRead(A1) , 0 , 1023 , 0 , 180);
  desplazamiento_msg.data = map(analogRead(A2) , 0 , 1023 , 0 , 5);

  publicador_angulo1.publish( &angulo1_msg );
  publicador_angulo2.publish( &angulo2_msg );
  publicador_desplazamiento.publish( &desplazamiento_msg );

  nodo_publicador_arduino.spinOnce();
  delay(100);
}
```

* Para la lectura de las se;ales publicadas en los topicos desde Arduino se puede utilizar el siguiente codigo en python para la presentacion en consola:

```
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback_angulo1(dato):
	print("angulo 1 recibido es: ")
	print(dato.data)

def callback_angulo2(dato):
	print("angulo 2 recibido es: ")
	print(dato.data)

def callback_desplazamiento(dato):
	print("el desplazamiento recibido es: ")
	print(dato.data)

def funcion_principal():
	rospy.Subscriber("Topic_angulo1_arduino" , Float32 , callback_angulo1)
	rospy.Subscriber("Topic_angulo2_arduino" , Float32 , callback_angulo2)
	rospy.Subscriber("Topic_desplazamiento_arduino" , Float32 , callback_desplazamiento)

	rospy.init_node('Nodo_suscriptor',anonymous=True)
	rospy.spin()

if __name__=='__main__':
	try:
		funcion_principal()
	except rospy.ROSInterruptException:
		pass

```
* Para el control del modelo URDF del robot de dos grados de libertad se puede usar el siguiente codigo en python, el cual se suscribira a los topicos donde se estan publicando las se;ales analogicas y publicara en el topico adecuado para el control de las juntas:
```
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math

def send_joint_angles():
    # Inicializar el nodo ROS
    rospy.init_node('joint_angles_publisher', anonymous=True)

    # Crear un publicador para enviar los ángulos de las articulaciones
    joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Frecuencia de publicación en Hz
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        # Crear un mensaje de JointState
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['rot1', 'rot2' , 'pri1']  # Nombres de las articulaciones
        joint_state_msg.position = [ 0.0 , 0.0 , 0.0]  # Ángulos de rotación en radianes

        angulo1 = rospy.wait_for_message('Topic_angulo1_arduino', Float32)
        angulo2 = rospy.wait_for_message('Topic_angulo2_arduino', Float32)
        desplazamiento = rospy.wait_for_message('Topic_desplazamiento_arduino', Float32)
        joint_state_msg.position[0] = math.radians(angulo1.data)
        joint_state_msg.position[1] = math.radians(angulo2.data)
        joint_state_msg.position[2] = desplazamiento.data/100

        # Publicar el mensaje
        joint_state_publisher.publish(joint_state_msg)

        # Dormir para mantener la frecuencia de publicación
        rate.sleep()

if __name__ == '__main__':
    try:
        send_joint_angles()
    except rospy.ROSInterruptException:
        pass
```

* Para la ejecucion del nodo rosserial y del archivo ".py" que publica el estado de las juntas para el modelo en RVIZ se deben agregar las siguientes lineas al archivo ".launch" que publica el modelo URDF:
```
   <!-- Ejecuta el nodo para rosserial -->
   <node name="serial_node" pkg="rosserial_python" type="serial_node.py" output="screen">
       <param name="port" value="/dev/ttyACM0" />
   </node>

   <!-- Ejecuta el nodo para controlar el modelo URDF -->
   <node name="Nodo_control" pkg="paquete_rviz" type="Control_rviz_arduino.py" output="screen" />
```
# Nodo suscriptor en Arduino
* El codigo de programacion que genera un nodo suscriptor en Arduino tiene la siguiente estructura:
```
#include <ros.h>
#include <std_msgs/UInt16.h>

#define LED 9

ros::NodeHandle node_handle;

std_msgs::UInt16 led_msg;

void subscriberCallback(const std_msgs::UInt16& led_msg) {
//  if (led_msg.data  == 1) {
//    digitalWrite(LED, HIGH);
//  } else {
//    digitalWrite(LED, LOW);
//  }
    analogWrite(LED,led_msg.data);
}

ros::Subscriber<std_msgs::UInt16> led_subscriber("intensidad_led", &subscriberCallback);

void setup(){
  pinMode(LED, OUTPUT);

  node_handle.initNode();
  node_handle.subscribe(led_subscriber);
}

void loop(){
  node_handle.spinOnce();
  delay(100);
}

* El codigo anterior se suscribe a un topico llamado *intensidad_led* en el cual se enviara un valor entero de 0 a 255 para controlar la intensidad de un led, dicho valor puede ser enviado con la ayuda de un archivo ".py" el cual tenga un nodo publicador con la siguiente estructura:

#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16

def talker():
	pub = rospy.Publisher('intensidad_led', UInt16 , queue_size=1)
	rospy.init_node('Nodo_publicador',anonymous=True)
	rate = rospy.Rate(1) # 1 Hz
	while not rospy.is_shutdown():
		variable = int(input("Ingrese la intensidad del led: "))
		pub.publish(variable)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

  # Mensajes Path
* Se realizo las siguientes modificaciones para agregar mensajes del tipo Path al movimiento del efector final del robot de dos grados de libertad:

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def envio_angulo_juntas():
    # Inicializar el nodo ROS
    rospy.init_node('Publicacion_angulos_juntas', anonymous=True)

    # Crear un publicador para enviar los ángulos de las articulaciones
    joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)


    path_publisher = rospy.Publisher('Topic_path', Path, queue_size=10)
    path= Path()

    # Frecuencia de publicación en Hz
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        # Crear un mensaje de JointState
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['rot1', 'rot2' , 'pri1']  # Nombres de las articulaciones
        joint_state_msg.position = [ 0.0 , 0.0 , 0.0]  # Ángulos de rotación en radianes


        angulo1 = rospy.wait_for_message('Topic_angulo1_arduino', Float32)
        angulo2 = rospy.wait_for_message('Topic_angulo2_arduino', Float32)
        desplazamiento = rospy.wait_for_message('Topic_desplazamiento_arduino', Float32)
        joint_state_msg.position[0] = math.radians(angulo1.data)
        joint_state_msg.position[1] = math.radians(angulo2.data)
        joint_state_msg.position[2] = desplazamiento.data / 100

        # Publicar el mensaje
        joint_state_publisher.publish(joint_state_msg)

        # Mensajes Path
        path.header.frame_id = 'base'     # Configurar el marco de referencia
        point1 = PoseStamped() 	   # Agregar puntos a la trayectoria
        point1.pose.position.x = 0.100*math.cos(math.radians(angulo1.data)) + 0.070*math.cos(math.radians(angulo1.data)+math.radians(angulo2.data))
        point1.pose.position.y = 0.050 + 0.100*math.sin(math.radians(angulo1.data)) + 0.070*math.sin(math.radians(angulo1.data)+math.radians(angulo2.data))
        point1.pose.position.z = desplazamiento.data / 100
        path.poses.append(point1)
        path_publisher.publish(path)

        # Frecuencia de publicación
        rate.sleep()

if __name__ == '__main__':
    try:
        envio_angulo_juntas()
    except rospy.ROSInterruptException:
        pass
```
        
