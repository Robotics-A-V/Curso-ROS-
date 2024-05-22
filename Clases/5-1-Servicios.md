creamos la carpeta srv

```
mkdir srv
```
y ejecutamos visual studio code

```
code .
```
* Dentro de la carpeta srv creamos un nuevo archivo con el nombre
"cinematicaI.srv"

```
cinematicaI.srv
```
* Agregamos la estructura del archivo

```
#Request
int64 x
int64 y
int64 z
---
#Response
float32[] angulos
int64 altura
```
* moficamos el archivo CMakelist.list de la siguiente forma:

 aproximadamente linea 59

Descomentamos la generacion de servicios
Agregamos el servicio
Descomentamos la generacion de mensajes


```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


 add_service_files(
    FILES
    cinematicaI.srv
 )

generate_messages(
DEPENDENCIES
std_msgs
 )
```


 en el archivo package.xml revisar que existe la linea de código:

```

    <build_depend>message_generation</build_depend>

```
* abrimos un nuevo terminal y colocamos:
```
  cd Curso_ros
  catkin_make
```
inicializamos nuestro entorno de ROS
```
roscore
```
colocamos para compronbar
```
rossrv show cinematicaI
```
nodo_server.py
```
#!/usr/bin/env python3                         
# encoding: utf-8

import rospy                                                                #Importamos ropsy (interface de python-ROS)
from std_msgs.msg import Float32MultiArray
from ros_23.srv import cinematicaI, cinematicaIResponse                     #Importamos módulos generados por nuestro servicio


def calculoCI(req):
    #Definimos para procesar la data enviada por el Cliente
    print ("Returning C.I. ")      
    #Imprimimos en pantalla los valores que recibimos
    #ecuaciones del caculo
    
    # respuesta
    angulos = Float32MultiArray()
    angulos.data = [req.x+0.2, req.y+0.3]
    
    return cinematicaIResponse(angulos.data, req.x,)                            #Retornamos al Cliente, el resultado de la suma de dos números enteros

def nodo():                                                                 #Definimos una función nodo                                   

    rospy.init_node('nodo_cinameticaI_server')                            #Inicializamos nuestro nodo y le asignamos un nombre = nodo_suma_two_ints_server

    #Declaramos nuestro Servicio Server    
                      #Nombre del servicio | Clase Servicio|Función para procesar la data enviada por el Cliente     
    s = rospy.Service('CinematicaI', cinematicaI, calculoCI)   

    print("Listo para calcular la cinemática inversa") #Imprimimos un mensaje en pantalla

    rospy.spin()                    #Mantiene corriendo el script hasta que se pulsa Crtl+C

if __name__ == '__main__':                                  
    try:
        nodo()                                                              # Lamamos a la función nodo
    except rospy.ROSInterruptException:                                     # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass                                                                            
 ```

nodo_cliente.py
```

#!/usr/bin/env python3                         
# encoding: utf-8

import rospy                              #Importamos ropsy (interface de python-ROS)
from ros_23.srv import cinematicaI        #Importamos el modulo generado por nuestro servicio

def cinameticaI_client(x, y, z):               #Definimos una función para enviar la data al Service Server y obtener el resultado
    rospy.wait_for_service('CinematicaI')  #Esperamos el servicio si no está listo                                                                
    try:
        #Definimos el Servicio Cliente en la variable add_two_ints
                                          #Nombre Servicio|Clase Servicio
        calculo = rospy.ServiceProxy('CinematicaI', cinematicaI)  
        resp = calculo(x, y, z)       #Enviamos la data para ser procesada en el Service Server


        # Continuo con el ejercicio



        # ------------------------
        return resp.altura , resp.angulos                #Retornamos el resultado de la operación
    except rospy.ServiceException as e :    #Si hay una excepción durante el procesamiento lo imprimimos
        print("Service call failed: %s"%e)

def nodo():                                       #Definimos una función nodo                                   

    rospy.init_node('cinematicaI_client')  #Inicializamos nuestro nodo y le asignamos un nombre = cinameticaI_client

    #Definimos dos variables x & y para realizar la suma de los dos números enteros
    x = 7                                         
    y = 8
    z = 3

    print("Requesting %s,%s,%s "%(x, y ,z ))           #Imprimimos en pantalla la posición requerida

    #Imprimimos el resultado de la operación de los dos números enteros
    #La operación de CI cinameticaI_client
    print(cinameticaI_client(x, y, z))


if __name__ == '__main__':                                  
    try:
        nodo()                               # Lamamos a la función nodo
    except rospy.ROSInterruptException:      # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass
```
