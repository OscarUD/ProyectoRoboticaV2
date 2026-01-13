# Robótica y Automatización Inteligente - Entorno de desarrollo ROS y demostrador de juegos por gestos

Este reporistorio proporciona, por un lado, un entorno de trabajo basado en ROS empaquetado en contendores Docker para la asignatura de Robótica y Automatización Inteligente. Por otro lado, contiene el codigo de un demostrador de interacción humano-robot en que varios usuarios juegan a Piedra-Papel-Tijera y Vaqueros mediantes gestos de la mano mientras un robot manipulador mueve bloques y fichas sobre una mesa.<br> 
El proyecto se apoya en Movelt! para aplanificar las trayeatorias, en nodos de visión artificial para el reconocimiento de gestos y localización de fichas, y en nodos especificos de control para coodinar el movimiento del brazo y la lógica de los juegos. 


## Carpeta de Videos - Google Drive
Este es el enlace para ver el vídeo de demostración:
[Video en Google Drive]https://drive.google.com/file/d/1quo6R7cpigFSMUeRCN0t47HbaDI6fEz3/view?usp=sharing

## Herramientas Utilizadas
En el host se requieren al menos las siguientes herramientas: 
1. Docker: ejecución de contendores.
2. Visual Studio Code + extensión *Dev Containers*: desarrollo direntamente dentro del contendor.
3. Git: colnación y gestión del repositorio
4. (Opcional, Linux con GPU NVIDIA) NVIDIA Container Toolkit para el perfil con aceleración gráfica. 

## Perfiles de contendor
El repositorio define varios pefiles de ejecución en *docker-compose* (pueden variar segun el fork)
|Pefil| Uso principal| Requisitos|
|-----|--------------| ----------| 
|desktop|Entorno de escrtotio vía web (no VNC) con ROS | Cualquier SO con Docker| 
| local| Entorno local sin GPU, pensado para laboratorio| Host Linux/WSL con servidor X |
|local gpu|Entorno local con soporte GPU (aceleración hardware) |Linux + X + NVIDIA + nvidia-container-toolkit| 

## Descripción de nodos ROS del proyecto 
A continuación se describe la parte específica del demostrados de juegos por gestos y manipulación de fichas. 

### Nodo WEBCAM (cámara jugadores y cámara robot)
- **Función**: adquiere en tiempo real las imágenes de la cámara USB orientada hacia los jugaodres y las publica como un flujo de imágenes en bruto.
- **Paquete**: nodo basado en *usb_cam* lanzado desde los ficheros *.launch* del proyecto
- **Topics**:
    - Publica: /usb_cam/image_raw
- **Uso**: tosos los nodos de visión que detectan manos y gestos se suscriben a este topic sin acceder directamente al dispositivo de vídeo físico.

### Nodo GESTOS (interacción y lógica de juegos)
- **Función**: módulo de interacción humano-maquina. Gestiona la captura de imagen de la cámara de jugaodres, detecta mano y clasfica los gestos. Permite seleccionar el juego mediante gestos y ejecuta la lógica completa de las partidad.
- **Estados Principales**:
  - *gameselector*: el usuario selecciona el juego apuntando a la cámara (1 dedo --> Vaqueros, 2 dedos --> PPTLS, puño --> salir)
  - *Vaqueros*: juego de disparos; cada jugador realiza gestos de Recargar, Disparar, Protegerse o Bomba, y el nodo actualiza las balas y puntaciones aplicando reglas definidas.
  - *PPTLS*: Juego de Piedra-Papel-Tijera-Lagarto-Spock; se gestiona una máquina de estado con cuenta atras, detección de gestos estables y detección de trampas (cambos tardíos de gesto o mano ausente)
- **Topics**:
    -   suscribe: /camara1/usb_cam/image_raw 
    -   publica: /juego (indicar que juego esta activo) & /ganador (indicar que jugador ha ganado cada ronda)
- **Integración robótica**: A partir de los gestos detectados, el nodo decide el resultado de las rondas y genera información (juego seleccionado, ganador de ronda) que consumirá el nodo de control del robot para mover fichas del color correspondiente.

### Nodo DETECTOR 
- **Función**: ecibe las imágenes de la cámara orientada a la zona de trabajo del robot, calcula la posición de las fichas rojas y azules en la mesa y publica esas posiciones para que el manipulador pueda planificar movimientos hacia ellas.
- **Topics**:
    - suscribe: /camara2/usb_cam/image_raw
    - publica: /coordenadas_rojas (coordenadas de las fichas rojas) , /coordenadas_azules (coordenadas de las fichas azules) & /coordenadas_verdes (coordenadas de las fichas verdes)
- **Uso**: Obtiene el estado actual del robot y añade objetos al plano para que se tengan en cuenta y evitar colisiones. Además carga las posiciones de referencia como la pose del marcador ArUco. 

### Nodo CENTRAL 
- **Función**: Actuar como núcleo de coordinación del sistema, conectando el módulo de gestos, el módulo de detección de fichas y el controlador del robot.
- **Topics**:
    - suscribe: /juego , /ganador (por parte del nodo gestos)  & /coordenadas_azule, /coordenadas_rojas , /coordenadas_verdes (por parte de la detección de fichas en el tablero)
- **Uso**: Orquestar el movimiento del manipulador: decidir qué ficha debe moverse (color del ganador), desde qué posición y hacia qué destino (pila de cubos o ábaco) y ejecutar las trayestorias correpondientes. Mantener el estado global de la demostración (ronda actual, número de fichas movidas, cambio de juego, reinicio de partida) y garantizar que la secuencia “detectar gestos → decidir ganador → mover robot” se ejecuta de forma ordenada y segura.

## Instalación y ejecución básica 
### 1. Clonar este repositorio
Clona este repositorio en tu máquina local:
```bash
git clone https://github.com/OscarUD/ProyectoRoboticaV2.git
```
### 2. Construir la imagen de los contenedores
Depende del contenedor que se quiera utilizar, construir uno u otro (`local`, `local_gpu`, `desktop`)
```bash
docker compose --profile <nombre_contenedor_elegido> build
```
Cuando acabe el proceso ya se dispondrá de la imagen construida en el sistema.

### 3. Lanzar el contenedor
Una vez construida la imagen es posible crear contenedores a partir de ella. En este caso los contenedores están diseñados para utilizarlos como entornos de desarrollo. Se recomienda crear y utilizar un solo contenedor. Para esto ejecutar:
```bash
docker compose --profile <nombre_contenedor_elegido> up
```
Una vez lanzado, la terminal se mantendrá ejecutando el contenedor hasta que se pare. Para pararlo basta con enviar una señal de terminación `ctrl`+`c`.

### 4. Acceso al contenedor
Es posible acceder al contenedor creado de varias maneras:
- Si el contenedor utilizado es el `desktop`, desde cualquier navegador se puede acceder a él en [esta dirección](http://localhost:6081). **La contraseña es laboratorio**. 
- Para todos los contenedores se recomienda asociar una instancia de VSCode al contenedor utilizando la extension "remote explorer". Esto permite desarrollar en VSCode como si se trabajara en local, pero ejecutando todo en el contenedor.
  
### 5. Primeros pasos en ROS
Dentro de los contenedores se incluye un espacio de trabajo de ROS con todos los elementos necesarios para poder trabajar con los robots del laboratorio.
Antes de poder utilizarlos es necesario construir el espacio de trabajo base. Para construir el espacio de trabajo:

1. Conectarse al contenedor desde una instancia de VSCode.
2. Abrir una nueva terminal en VSCode.
3. Navegar hasta el directorio base del espacio de trabajo:
```bash
cd /home/laboratorio/ros_workspace
```
4. Actualizar la lista de paquetes disponibles del sistema:
```bash
sudo apt update
```
5. Actualizar el gestor de paquetes de ROS:
```bash
rosdep update --include-eol-distros
```
6. Instalar todas las dependencias del espacio de trabajo base:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
7. Construir el espacio de trabajo:
```bash
catkin build
```
Si todo se ha ejecutado de manera correcta, el espacio de trabajo ya está en condiciones de uso.

Antes de poder utilizar el espacio de trabajo hay que activarlo en la terminal:
```bash
source /home/laboratorio/ros_workspace/devel/setup.bash
```
>**IMPORTANTE:**
**<u>La activación del espacio de trabajo se debe realizar en cada nueva terminal abierta en la que se quiera utilizar algo relacionado con este.</u>**

Si siempre se va a utilizar el mismo espacio de trabajo, es posible incluir la activación en el fichero `/home/laboratorio/.bashrc` que se ejecuta cada vez que se abre una nueva terminal. Si se incluye, no hace falta volver a activar el espacio de trabajo.

Para comprobar que el sistema funciona correctamente, ejecutar en la terminal donde se ha activado el espacio de trabajo:
```bash
roslaunch launcher_robots_lab_robotica sim_203.launch
```
Esto debería ejecutar todos los nodos necesarios para poder controlar uno de los Universal Robots en simulación. Debería aparecer una interfaz gráfica en la que se muestra el robot y si se selecciona el grupo de planificación `robot`, debería permitir moverlo a diferentes posiciones, planificar y ejecutar trayectorias.

