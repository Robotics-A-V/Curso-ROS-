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
