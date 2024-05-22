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
