# Creacion de un espacio de trabajo
* Creamos la carpeta de un espacio de trabajo con el siguiente comando:

```
mkdir [Nombre_del_espacio_de_trabajo]
```
* Creamos una subcarpeta llamada src dentro del espacio de trabajo

```
cd [Nombre_del_espacio_de_trabajo]
mkdir src
```
* Compilamos dentro del espacio de trabajo con:

```
catkin_make
```
* Agregamos el nombre del espacio de trabajo creado en la parte final del archivo .bashrc con el sigunte comando:

```
source ~/[Nombre_del_espacio_de_trabajo]/devel/setup.bash
```
* El archivo .bashrc se lo encuentra en la carpeta personal del usuario, activando la visibilidad de los archivos ocultos.
