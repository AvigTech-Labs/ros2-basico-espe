# Instalación de ROS2 Humble

Humble es la versión de ROS2 más utilizada actualmente y cuenta con un soporte oficial hasta el año 2027.

La instalación se realizará usando la paquetería deb y en el siguiente enlace se encuentra su repositorio oficial. 

link oficial [ROS2-Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Pasos de instalación:

1. **Fuentes de configuración**

Añadir el repositorio ROS2 apt a su sistema.

Primero asegúdese de que el repositorio de Ubuntu Universe esté habilitado.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

2. **Configurar los repositorios de Ubuntu**

ROS 2 depende de algunas herramientas adicionales. Primero, debe asegurarse de que su sistema esté completamente actualizado y tenga las herramientas necesarias.

```
sudo apt update && sudo apt install curl -y
```
ahora es necesario agregar la clave GPG para ROS 2:
```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
y agregar el repositorio de ROS 2 Humble a su lista de fuentes:
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

3.**Instalar ROS 2 Humble**
Actualizar el indice de contenidos
```
sudo apt update
```
Actualizar el sistema
```
sudo apt upgrade
```
Instalacion de ROS HUMBLE
```
sudo apt install ros-humble-desktop
```
para poder utilizar las herramientas de construccion de paquetes es necesario instalar las herrmientas adicionales utilizando el siguietne comando 
```
sudo apt install python3-colcon-common-extensions
```
```
sudo apt install ros-dev-tools
```

**5. Configurar el entorno de ROS 2**

Para que ROS 2 funcione correctamente, debes agregar su entorno a tu terminal. Esto lo puedes hacer editando el archivo ~/.bashrc o ejecutando este comando en cada nueva terminal que abras:

bash
```
source /opt/ros/humble/setup.bash
```

Para automatizarlo en futuras sesiones, agrega el comando anterior al final del archivo ~/.bashrc:

```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**6. Verificar la instalación de ROS 2**

Para asegurarte de que ROS 2 se instaló correctamente, puedes intentar correr un nodo de prueba. Primero, abre una terminal nueva y ejecuta:

Luego, inicia una instancia del nodo talker (publicador):
```
ros2 run demo_nodes_cpp talker
```
Abre una nueva terminal, ejecuta nuevamente el source, y luego inicia una instancia del nodo listener (suscriptor):

```
ros2 run demo_nodes_cpp listener
```
Si todo está bien, verás mensajes del talker y listener interactuando en la terminal.

