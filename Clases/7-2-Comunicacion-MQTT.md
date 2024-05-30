# Instalación 

Abrimos la terminal y ejecutamos 

```
sudo apt update
```
```
sudo apt install mosquitto mosquitto-clients
```
```
sudo systemctl start mosquitto
```
```
sudo systemctl enable mosquitto

```

comprobamos que está ejecutandose
```
sudo systemctl status mosquitto
```

Instalar paho-mqtt
```
pip install paho-mqtt
```

# MQTT y JSON
Los siguientes programas contienen los comandos para el intercambio de mensajes usando el protocolo MQTT y el formato de mensajes JSON.

1. Abrir una termina en la carpeta donde se encuentren los archivos

2. Ejecutar los programas en diferentes terminales



```
python3 Equipo1.py
python3 Equipo2.py
```

Equipo 1



```
import paho.mqtt.client as mqtt
import json
import time

# Variables globales
global diccionario

# Configuración del cliente MQTT
broker_address = "localhost"
client = mqtt.Client("Central_Nodo")
client.connect(broker_address)

# Función que se ejecuta cuando llega un mensaje
def on_message(client, userdata, message):
    global diccionario
    # Obtener el mensaje como cadena de texto
    mensaje_recibido = str(message.payload.decode("utf-8"))
    # Convertir el mensaje JSON en un diccionario
    diccionario = json.loads(mensaje_recibido)
    # Imprimir el diccionario recibido
    print("Mensaje recibido:", diccionario)
    if(message.topic == "datos_2"):
        print(diccionario["robot1"])

# Configurar la función "on_message" como callback para el cliente MQTT
client.on_message = on_message
# Suscribirse al topic "datos_2"
client.subscribe("datos_2")

# Iniciar el bucle para manejar los eventos del cliente MQTT
# Bucle para enviar mensajes
while True:
    client.loop_start() # start the loop
    # Mensaje a enviar como diccionario
    mensaje = {"sensores": {
                            "Temperatura": "activado"

                            },
              "actuadores":"funcionando"       
               }
    # Convertir el diccionario en JSON
    mensaje_json = json.dumps(mensaje)
    # Publicar el mensaje en el topic "datos"
    client.publish("datos_1", mensaje_json)
    time.sleep(1)
    client.loop_stop()
```

Equipo 2



```
import paho.mqtt.client as mqtt
import json
import time


# Configuración del cliente MQTT
broker_address = "localhost"
client = mqtt.Client("cliente2")
client.connect(broker_address)

# Función que se ejecuta cuando llega un mensaje
def on_message(client, userdata, message):
    # Obtener el mensaje como cadena de texto
    mensaje_recibido = str(message.payload.decode("utf-8"))
    # Convertir el mensaje JSON en un diccionario
    sensores = json.loads(mensaje_recibido)
    # Imprimir el diccionario recibido
    print("Mensaje recibido:", sensores)
    if(message.topic == "datos_1"):
        print(sensores["sensores"]["Temperatura"])
    

# Configurar la función "on_message" como callback para el cliente MQTT
client.on_message = on_message
# Suscribirse al topic "datos"
client.subscribe("datos_1")

# Iniciar el bucle para manejar los eventos del cliente MQTT


# Bucle para enviar mensajes
while True:
    client.loop_start() # start the loop
    # Mensaje a enviar como diccionario
    mensaje = {"robot1": 0.256,
               "robot2": 0.35
               }
    # Convertir el diccionario en JSON
    mensaje_json = json.dumps(mensaje)
    # Publicar el mensaje en el topic "datos"
    client.publish("datos_2", mensaje_json)
    # Detener el bucle del cliente MQTT
    time.sleep(1)
    client.loop_stop()
```

# MQTT y ESP32

1. Abrir el archivo "Mqtt_Json.ino", contiene 2 pestañas.
2. Ejecutar el archivo "Equipo2.py"

Archivo ESP32.

1. Cambiar la contraseña y nombre de la red
2. Colocar la ip del equipo, para saber la ip que tiene su equipo coloque en la terminal de ubuntu el comando



```
ifconfig
```
 Pestaña 1
```
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "CARMEN GONZALEZ_";  //CARMEN GONZALEZ_
const char* password = "123wa321vg";    //123wa321vg

// Credenciales de MQTT
const char* mqtt_broker         = "192.168.100.76";
const int mqtt_port             = 1883;
const char* mqtt_topic_publicar = "datos_1";

// Objeto de cliente WiFi
WiFiClient espClient;
// Objeto de cliente MQTT
PubSubClient mqttClient(espClient);
// Tamaño máximo de la cadena JSON
const size_t capacidad_json = JSON_OBJECT_SIZE(10);


void setup() {
  // Inicializar puerto serial
  Serial.begin(115200);

  // Conectar a WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conexión WiFi establecida.");

  // Configurar cliente MQTT
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(callback);

  // Conectar al broker MQTT
  while (!mqttClient.connected()) {
    if (mqttClient.connect("esp32_client")){
      Serial.println("Conexión MQTT establecida.");
      mqttClient.subscribe("datos_2");
    } else {
      Serial.println("Error al conectar al broker MQTT, volveré a intentarlo en 5 segundos.");
      delay(5000);
    }
  }
}

void loop() {
  // Verificar si se recibió algún mensaje MQTT
  mqttClient.loop();
  
}
```

Pestaña  2

```
void callback(char* topic, byte* payload, unsigned int length) {
  // Crear objeto JSON
  StaticJsonDocument<capacidad_json> doc;

  // Copiar payload a un buffer
  char buffer[length];
  memcpy(buffer, payload, length);

  // Deserializar JSON
  DeserializationError error = deserializeJson(doc, buffer, length);

  // Verificar errores de deserialización
  if (error) {
    Serial.print("Error al deserializar JSON: ");
    Serial.println(error.c_str());
    return;
  }
  // Obtener valores del JSON
  float movil1 = doc["robot1"];
  
  // Imprimir valores recibidos
  Serial.print("Velocidad del robot 1: ");
  Serial.println(movil1);

  // Enviar mensajes

  DynamicJsonDocument mensaje(256);
  mensaje["sensores"]["Temperatura"]= 0.1234;
  // Convertir el objeto JsonDocument en una cadena JSON
  String mensaje_json;
  serializeJson(mensaje, mensaje_json);

  // Publicar el mensaje en el topic "datos"
  mqttClient.publish(mqtt_topic_publicar, mensaje_json.c_str(),0);
}
```


