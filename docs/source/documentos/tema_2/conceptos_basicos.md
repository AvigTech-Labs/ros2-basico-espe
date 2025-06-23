# Conceptos Básicos

## Espacio de trabajo

### ¿Qué es un espacio de trabajo en ROS 2?

Un **espacio de trabajo (workspace)** en ROS 2 es una carpeta estructurada que contiene paquetes ROS 2, los cuales pueden estar escritos en Python o C++. Todos los paquetes se compilan y organizan dentro del workspace.


***¿Qué es colcon?***

`colcon` es la herramienta oficial y recomendada para **construir, compilar y organizar workspaces en ROS 2**.

Proviene de "COmplex LOgic CONstructor" y fue diseñado para reemplazar herramientas anteriores como `catkin_make` o `ament_tools`.

---
***Características de colcon***

- Compila múltiples paquetes de forma **paralela**.
- Detecta **dependencias entre paquetes** y respeta el orden de compilación.
- Separa los archivos construidos en carpetas específicas.
- Es **extensible** mediante plugins.

---

Instalación de colcon y extensiones recomendadas

```bash
sudo apt install python3-colcon-common-extensions
```

---

***Comandos principales de colcon***

| Comando                          | ¿Qué hace? |
|----------------------------------|------------|
| `colcon build`                   | Compila todos los paquetes en `src/` |
| `colcon build --packages-select nombre` | Compila solo un paquete específico |
| `colcon list`                    | Muestra todos los paquetes del workspace |
| `colcon test`                    | Ejecuta pruebas definidas en los paquetes |
| `colcon clean`                   | Limpia las carpetas `build/`, `install/` y `log/` |
| `colcon build --symlink-install`| En paquetes Python, enlaza código para no recompilar cada vez |

---

***Directorios creados por colcon al compilar***

Al ejecutar `colcon build`, se generan 3 carpetas principales:

```
ros2_ws/
├── build/
├── install/
└── log/
```

| Carpeta   | ¿Para qué sirve? |
|-----------|------------------|
| `build/`  | Archivos temporales de compilación. ROS 2 genera aquí el código intermedio. |
| `install/`| Aquí se instalan los ejecutables, scripts, bibliotecas y archivos generados. Es lo que usas con `source install/setup.bash`. |
| `log/`    | Archivos de salida, advertencias y errores de compilación. Útil para depuración. |

---

***Limpieza de la build***

Si es necesario limpiar el workspace por completo:

```bash
colcon build --packages-select my_package --cmake-clean-cache
```

O para limpiar todo:

```bash
rm -rf build/ install/ log/
```
---


## Paquetes

***¿Qué son los paquetes?***

Un paquete es la unidad mínima de construcción en ROS 2. Contiene código fuente, archivos de lanzamiento, configuraciones, y puede ser en Python o C++.

¿Qué tipos de paquetes existen?

- `ament_cmake` → para paquetes en C++
- `ament_python` → para paquetes en Python

***Diferencias entre paquetes en Python y C++***

| Característica | C++ (`ament_cmake`) | Python (`ament_python`) |
|----------------|---------------------|--------------------------|
| Compilación    | Requiere CMake      | No necesita compilar código |
| setup.py       | No usa              | Obligatorio              |
| Velocidad      | Más rápida          | Más simple de escribir   |

---

***Crear un paquete Python desde cero***

Para crear un paquete usamos el comando `ros2 pkg create` y añadiños las siguientes argumentos :

***Descripción de opciones:***

| Parte | Función |
|-------|---------|
| `my_package` | Nombre del paquete |
| `--build-type` | Define el tipo de compilador |
| `--license` | Agrega la licencia al package.xml |
| `--node-name` | No crea nodo por defecto, se ignora usualmente |
| `--description` | Agrega la descripción del paquete, se ignora usualmente |

---

***Resumen para crear un paquete Python completo***

```bash
ros2 pkg create py_listener_pkg \
  --build-type ament_python \
  --dependencies rclpy std_msgs \
  --license "MIT" \
  --description "Paquete de ejemplo en Python para suscripción ROS 2"
```
---


## Nodos

Un **nodo** es una entidad computacional que ejecuta una función específica, como mover una tortuga, leer sensores, o calcular trayectorias.

Ejemplo:
- `turtlesim_node`: renderiza la tortuga y ejecuta sus acciones.
- `turtle_teleop_key`: interpreta teclas y publica comandos de velocidad.
- `teleop_twist_keyboard`: interpreta teclas y publica comandos de velocidad.

Listar nodos activos:

```bash
ros2 node list
```
---


## Temas

Un **tema** es un canal por el cual los nodos publican o reciben datos. Es **unidireccional y asincrónico**.

Ejemplo:
- `/turtle1/cmd_vel`: donde se publican los comandos de velocidad lineal y angular.
- `/turtle1/pose`: donde se publica la posición actual de la tortuga.

Ver temas activos:

```bash
ros2 topic list
```
---

![Seleccion](./temas.gif)

## Ejemplos de Lanzamiento y Control de Turtlesim en ROS 2

En este ejemplo se muestra cómo lanzar el nodo de `turtlesim` y controlar su movimiento mediante el teclado. También se explica cómo utilizar otro nodo de teleoperación genérico (`teleop_twist_keyboard`) haciendo uso de **remapeo de temas** en ROS 2.


***Paso 1: Instalación de paquetes necesarios***

```bash
sudo apt update
sudo apt install ros-humble-turtlesim ros-humble-teleop-twist-keyboard
```

***Paso 2: Lanzar la ventana del simulador***

```bash
ros2 run turtlesim turtlesim_node
```

Este comando lanza el nodo que genera la ventana de simulación con la tortuga inicial (`/turtle1`).

***Paso 3: Controlar la tortuga con el teclado (teleop integrado)***

```bash
ros2 run turtlesim turtle_teleop_key
```
---

***Paso 4: Control con un nodo genérico de teleoperación***

En este ejemplo se usa el nodo `teleop_twist_keyboard` del paquete `teleop_twist_keyboard`. Este nodo:
- Lee las teclas del teclado
- Publica mensajes del tipo `geometry_msgs/msg/Twist`

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Por defecto, este nodo publica en el tema:

```
/cmd_vel
```

Sin embargo, en `turtlesim`, la tortuga escucha comandos en:

```
/turtle1/cmd_vel
```
---

***¿Cómo solucionar esto?***

### Remapeo de temas

ROS 2 permite redirigir ("remapear") los nombres de los temas **sin necesidad de cambiar el código** del nodo. Para hacerlo se usa:

```bash
--ros-args -r [tema_origen]:=[tema_destino]
```

En este caso específico:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/turtle1/cmd_vel
```

Esto se entiende como:

> "Cuando este nodo publique en `/cmd_vel`, en realidad envíalo a `/turtle1/cmd_vel`."

---

¿Por qué es útil el remapeo?

- Evita modificar el código fuente de los nodos.
- Permite **reutilizar nodos genéricos** con distintos robots o simuladores.
- Facilita pruebas rápidas con distintos sistemas.






# Tutoriales

## Creación de un espacio de trabajo

A continuación, se realiza la creación de un espacio de trabajo denominado `ros2_ws`

***Creación del espacio de trabajo***

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
---


## Creación Paquetes

Una vez construido el espacio de trabajo, se debe realizar la construcción de un paquete de ROS2, en este caso se utilizará un paquete compatible con python, y tendrá las dependencias de `std_msgs`

1. Crear el paquete

```bash
cd ~/ros2_ws/src
ros2 pkg create mi_pkg_python --build-type ament_python --dependencies rclpy std_msgs
```

2. Estructura generada

```
mi_pkg_python/
├── mi_pkg_python/
│   └── __init__.py
├── package.xml
├── resource/
│   └── mi_pkg_python
├── setup.py
├── setup.cfg
└── test/
```

3. Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select mi_pkg_python
```
## Nodos publicador y Suscriptor
Una vez compilado el paquete, se realiza la creación de los nodos publicador y suscriptor.

1. Crear el archivo del nodo publicador

```bash
touch mi_pkg_python/mi_pkg_python/publicador.py
```

2. Código del nodo en `publicador.py`

```python
# Importa la librería principal de ROS 2 en Python
import rclpy

# Importa la clase base Node, que representa un nodo en ROS 2
from rclpy.node import Node

# Importa el tipo de mensaje estándar String del paquete std_msgs
from std_msgs.msg import String

# Define una clase que extiende Node, representando un nodo que publica mensajes
class MinimalPublisher(Node):

    def __init__(self):
        # Llama al constructor de la clase padre (Node) con el nombre del nodo
        super().__init__('minimal_publisher')

        # Crea un publicador que publica mensajes del tipo String en el topic 'topic' con una cola de 10 mensajes
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Crea un temporizador que ejecuta la función `timer_callback` cada 0.5 segundos
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Contador para numerar los mensajes
        self.i = 0

    # Función que se ejecuta cada 0.5 segundos
    def timer_callback(self):
        # Crea un mensaje tipo String
        msg = String()

        # Asigna el contenido del mensaje
        msg.data = f'Hello World: {self.i}'

        # Publica el mensaje en el topic
        self.publisher_.publish(msg)

        # Muestra por consola el mensaje publicado
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Incrementa el contador
        self.i += 1

# Punto de entrada principal del programa
def main(args=None):
    # Inicializa el sistema de nodos de ROS 2
    rclpy.init(args=args)

    try:
        # Instancia el nodo y empieza a ejecutarlo
        node = MinimalPublisher()

        # Mantiene al nodo activo, escuchando eventos y timers
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Permite salir con Ctrl+C sin error
        pass

    finally:
        # Destruye el nodo de forma limpia y apaga el sistema ROS 2
        node.destroy_node()
        rclpy.shutdown()

# Verifica si el archivo se está ejecutando directamente (no importado)
if __name__ == '__main__':
    main()
```

3. Crear el nodo `suscriptor.py` en la misma dirección donde se encuentra el nodo publicador.py

```python

# Importa la librería principal de ROS 2 para Python
import rclpy

# Importa la clase base para crear nodos en ROS 2
from rclpy.node import Node

# Importa el tipo de mensaje estándar String desde std_msgs
from std_msgs.msg import String

# Define una clase que hereda de Node, representa el nodo suscriptor
class MinimalSubscriber(Node):

    # Método constructor de la clase
    def __init__(self):
        # Inicializa el nodo con el nombre 'minimal_subscriber'
        super().__init__('minimal_subscriber')

        # Crea una suscripción al tópico 'topic'
        # Tipo de mensaje: String
        # Función de callback: self.listener_callback
        # Tamaño de la cola: 10 mensajes
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Esta línea asegura que la variable no sea optimizada por el compilador (aunque no es obligatorio en Python)
        self.subscription

    # Función que se llama cada vez que se recibe un mensaje en el tópico
    def listener_callback(self, msg):
        # Imprime en consola el contenido del mensaje recibido
        self.get_logger().info('I heard: "%s"' % msg.data)

# Función principal que lanza el nodo
def main(args=None):
    # Inicializa el sistema de ROS 2
    rclpy.init(args=args)

    # Crea una instancia del nodo suscriptor
    minimal_subscriber = MinimalSubscriber()

    # Mantiene el nodo corriendo escuchando datos (callback se ejecutará cuando lleguen mensajes)
    rclpy.spin(minimal_subscriber)

    # Cuando se detiene (Ctrl+C o cierre), se destruye el nodo explícitamente (opcional)
    minimal_subscriber.destroy_node()

    # Finaliza la ejecución del sistema ROS 2
    rclpy.shutdown()

# Esta verificación permite que el nodo se ejecute solo si el script se corre directamente
if __name__ == '__main__':
    main()
```

4. Registrar los nodos en `setup.py`

```python
entry_points={
    'console_scripts': [
        'publicador1 = mi_pkg_python.publicador:main',
        'suscriptor1 = mi_pkg_python.suscriptor:main',
    ],
},
```

5. Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select mi_pkg_python
```

6. Ejecutar el nodo

```bash
ros2 run mi_pkg_python publicador1
```
