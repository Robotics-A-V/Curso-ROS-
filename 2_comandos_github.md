**1. Configuración Inicial**

Configurar tu identidad
```
git config --global user.name "Tu Nombre"
git config --global user.email "tuemail@example.com"
```
Comandos Básicos de Git
Inicializar un repositorio
```
git init
```
Clonar un repositorio
```
git clone https://github.com/usuario/repositorio.git
```
Agregar archivos al área de preparación
```
git add archivo.txt
# O para agregar todos los archivos
git add .
```
Confirmar cambios
```
git commit -m "Mensaje del commit"
```
Ver el estado del repositorio
```
git status
```
Ver el historial de commits
```
git log
```
**2. Trabajo con Ramas**

Crear una nueva rama
```
git branch nombre-de-la-rama
```
Cambiar de rama
```
git checkout nombre-de-la-rama
```
Crear y cambiar a una nueva rama
```
git checkout -b nombre-de-la-rama
```
Fusionar ramas
```
git checkout rama-principal
git merge nombre-de-la-rama
```
**3. Sincronización con GitHub**

Agregar un repositorio remoto
```
git remote add origin https://github.com/usuario/repositorio.git
```
Verificar repositorios remotos
```
git remote -v
```
Obtener cambios del repositorio remoto
```
git pull origin rama-principal
```
Enviar cambios al repositorio remoto
```
git push origin rama-principal
```
**4. Otros Comandos Útiles**

Eliminar un archivo
```
git rm archivo.txt
git commit -m "Eliminar archivo.txt"
git push origin rama-principal
```
Restaurar archivos modificados pero no confirmados
```
git checkout -- archivo.txt
```
Cancelar un commit antes de hacer push

```
git reset --soft HEAD~1
# Si quieres deshacer también los cambios locales, usa:
git reset --hard HEAD~1
```
