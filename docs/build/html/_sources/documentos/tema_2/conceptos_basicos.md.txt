# Conceptos Básicos

## Paquetes en ROS2

Un **paquete** en ROS 2 es una carpeta estructurada que agrupa:
- Nodos ejecutables
- Mensajes y servicios personalizados
- Archivos de configuración y lanzamiento

Los paquetes permiten organizar y reutilizar componentes robóticos. ROS dispone de herramientas (`ros2 pkg`) para buscarlos, instalarlos y ejecutarlos.
---

## Nodos en ROS2

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

