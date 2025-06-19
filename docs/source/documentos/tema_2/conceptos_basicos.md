# Conceptos Básicos

## Uso de paquetes en ROS2

Un **paquete** en ROS 2 es una carpeta estructurada que agrupa:
- Nodos ejecutables
- Mensajes y servicios personalizados
- Archivos de configuración y lanzamiento

Los paquetes permiten organizar y reutilizar componentes robóticos. ROS dispone de herramientas (`ros2 pkg`) para buscarlos, instalarlos y ejecutarlos.

## Uso de nodos en ROS2

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

## Uso Temas

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


## Creación de un espacio de trabajo

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

***Creación del espacio de trabajo***

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

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


## Creación Paquetes

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

```bash
cd ~/ros2_ws/src
ros2 pkg create my_package --build-type ament_python --license Apache-2.0 --node-name my_node
```

***Descripción de opciones:***

| Parte | Función |
|-------|---------|
| `my_package` | Nombre del paquete |
| `--build-type ament_python` | Define que es un paquete en Python |
| `--license Apache-2.0` | Agrega la licencia al package.xml |
| `--node-name` | No crea nodo por defecto, se ignora usualmente |

---

***Construir un paquete específico***

```bash
colcon build --packages-select my_package
```

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

### Ejemplo completo paso a paso

1. Crear el paquete Python

```bash
cd ~/ros2_ws/src
ros2 pkg create my_package --build-type ament_python --dependencies rclpy std_msgs
```

2. Estructura generada

```
my_package/
├── my_package/
│   └── __init__.py
├── package.xml
├── resource/
│   └── my_package
├── setup.py
├── setup.cfg
└── test/
```

3. Crear el archivo del nodo publicador

```bash
touch my_package/publisher.py
```

4. Código del nodo en `publisher.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MinimalPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

5. Registrar el nodo en `setup.py`

```python
entry_points={
    'console_scripts': [
        'publisher_node = my_package.publisher:main',
    ],
},
```

6. Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash
```

7. Ejecutar el nodo

```bash
ros2 run my_package publisher_node
```
