Conceptos Básicos
=================

Espacio de trabajo
------------------

¿Qué es un espacio de trabajo en ROS 2?

Un **espacio de trabajo (workspace)** en ROS 2 es una carpeta
estructurada que contiene paquetes ROS 2, los cuales pueden estar
escritos en Python o C++. Todos los paquetes se compilan y organizan
dentro del workspace.

**¿Qué es colcon?**

``colcon`` es la herramienta oficial y recomendada para **construir,
compilar y organizar workspaces en ROS 2**.

Proviene de “COmplex LOgic CONstructor” y fue diseñado para reemplazar
herramientas anteriores como ``catkin_make`` o ``ament_tools``.

--------------

**Características de colcon**

- Compila múltiples paquetes de forma **paralela**.
- Detecta **dependencias entre paquetes** y respeta el orden de
  compilación.
- Separa los archivos construidos en carpetas específicas.
- Es **extensible** mediante plugins.

--------------

Instalación de colcon y extensiones recomendadas

.. code:: bash

   sudo apt install python3-colcon-common-extensions

--------------

**Comandos principales de colcon**

+----------------------------------------------------+-----------------+
| Comando                                            | ¿Qué hace?      |
+====================================================+=================+
| ``colcon build``                                   | Compila todos   |
|                                                    | los paquetes en |
|                                                    | ``src/``        |
+----------------------------------------------------+-----------------+
| ``colcon build --packages-select nombre``          | Compila solo un |
|                                                    | paquete         |
|                                                    | específico      |
+----------------------------------------------------+-----------------+
| ``colcon list``                                    | Muestra todos   |
|                                                    | los paquetes    |
|                                                    | del workspace   |
+----------------------------------------------------+-----------------+
| ``colcon test``                                    | Ejecuta pruebas |
|                                                    | definidas en    |
|                                                    | los paquetes    |
+----------------------------------------------------+-----------------+
| ``colcon build --symlink-install``                 | En paquetes     |
|                                                    | Python, enlaza  |
|                                                    | código para no  |
|                                                    | recompilar cada |
|                                                    | vez             |
+----------------------------------------------------+-----------------+

--------------

**Directorios creados por colcon al compilar**

Al ejecutar ``colcon build``, se generan 3 carpetas principales:

::

   ros2_ws/
   ├── build/
   ├── install/
   └── log/

+--------------------------+-------------------------------------------+
| Carpeta                  | ¿Para qué sirve?                          |
+==========================+===========================================+
| ``build/``               | Archivos temporales de compilación. ROS 2 |
|                          | genera aquí el código intermedio.         |
+--------------------------+-------------------------------------------+
| ``install/``             | Aquí se instalan los ejecutables,         |
|                          | scripts, bibliotecas y archivos           |
|                          | generados. Es lo que usas con             |
|                          | ``source install/setup.bash``.            |
+--------------------------+-------------------------------------------+
| ``log/``                 | Archivos de salida, advertencias y        |
|                          | errores de compilación. Útil para         |
|                          | depuración.                               |
+--------------------------+-------------------------------------------+

--------------

**Limpieza de la build**

Si es necesario limpiar el workspace por completo:

.. code:: bash

   colcon build --packages-select my_package --cmake-clean-cache

O para limpiar todo:

.. code:: bash

   rm -rf build/ install/ log/

--------------

Paquetes
--------

**¿Qué son los paquetes?**

Un paquete es la unidad mínima de construcción en ROS 2. Contiene código
fuente, archivos de lanzamiento, configuraciones, y puede ser en Python
o C++.

¿Qué tipos de paquetes existen?

- ``ament_cmake`` → para paquetes en C++
- ``ament_python`` → para paquetes en Python

**Diferencias entre paquetes en Python y C++**

============== ===================== ===========================
Característica C++ (``ament_cmake``) Python (``ament_python``)
============== ===================== ===========================
Compilación    Requiere CMake        No necesita compilar código
setup.py       No usa                Obligatorio
Velocidad      Más rápida            Más simple de escribir
============== ===================== ===========================

--------------

**Crear un paquete Python desde cero**

Para crear un paquete usamos el comando ``ros2 pkg create`` y añadiños
las siguientes argumentos :

**Descripción de opciones:**

+------------------------------+---------------------------------------+
| Parte                        | Función                               |
+==============================+=======================================+
| ``my_package``               | Nombre del paquete                    |
+------------------------------+---------------------------------------+
| ``--build-type``             | Define el tipo de compilador          |
+------------------------------+---------------------------------------+
| ``--license``                | Agrega la licencia al package.xml     |
+------------------------------+---------------------------------------+
| ``--node-name``              | No crea nodo por defecto, se ignora   |
|                              | usualmente                            |
+------------------------------+---------------------------------------+
| ``--description``            | Agrega la descripción del paquete, se |
|                              | ignora usualmente                     |
+------------------------------+---------------------------------------+

--------------

**Resumen para crear un paquete Python completo**

.. code:: bash

   ros2 pkg create py_listener_pkg \
     --build-type ament_python \
     --dependencies rclpy std_msgs \
     --license "MIT" \
     --description "Paquete de ejemplo en Python para suscripción ROS 2"

--------------

Nodos
-----

Un **nodo** es una entidad computacional que ejecuta una función
específica, como mover un robot móvil simulado, leer sensores, o calcular
trayectorias.

Ejemplo: 

- ``turtlesim_node``: renderiza la tortuga y ejecuta sus acciones. 
- ``turtle_teleop_key``: interpreta teclas y publica comandos de velocidad. 
- ``teleop_twist_keyboard``: interpreta teclas y publica comandos de velocidad.

Listar nodos activos:

.. code:: bash

   ros2 node list

--------------

Temas
-----

Un **tema** es un canal por el cual los nodos publican o reciben datos.
Es **unidireccional y asincrónico**.

Ejemplo: 

- ``/turtle1/cmd_vel``: donde se publican los comandos de velocidad lineal y angular. 

- ``/turtle1/pose``: donde se publica la posición actual de la tortuga.

Ver temas activos:

.. code:: bash

   ros2 topic list

--------------

.. figure:: ./temas.gif
   :alt: Seleccion

   Tomado de: ROS2 Humble - Documentación Oficial

Ejemplos de Lanzamiento y Control de Turtlesim en ROS 2
-------------------------------------------------------

En este ejemplo se muestra cómo lanzar el nodo de ``turtlesim`` y
controlar su movimiento mediante el teclado. También se explica cómo
utilizar otro nodo de teleoperación genérico (``teleop_twist_keyboard``)
haciendo uso de **remapeo de temas** en ROS 2.

**Paso 1: Instalación de paquetes necesarios**

.. code:: bash

   sudo apt update
   sudo apt install ros-humble-turtlesim ros-humble-teleop-twist-keyboard

**Paso 2: Lanzar la ventana del simulador**

.. code:: bash

   ros2 run turtlesim turtlesim_node

Este comando lanza el nodo que genera la ventana de simulación con la
tortuga inicial (``/turtle1``).

**Paso 3: Controlar la tortuga con el teclado (teleop integrado)**

.. code:: bash

   ros2 run turtlesim turtle_teleop_key

--------------

**Paso 4: Control con un nodo genérico de teleoperación**

En este ejemplo se usa el nodo ``teleop_twist_keyboard`` del paquete
``teleop_twist_keyboard``. Este nodo: Lee las teclas del teclado y 
publica mensajes del tipo ``geometry_msgs/msg/Twist``

.. code:: bash

   ros2 run teleop_twist_keyboard teleop_twist_keyboard

Por defecto, este nodo publica en el tema:

::

   /cmd_vel

Sin embargo, en ``turtlesim``, la tortuga escucha comandos en:

::

   /turtle1/cmd_vel

--------------

**¿Cómo solucionar esto?**

Remapeo de temas
~~~~~~~~~~~~~~~~

ROS 2 permite redirigir (“remapear”) los nombres de los temas **sin
necesidad de cambiar el código** del nodo. Para hacerlo se usa:

.. code:: bash

   --ros-args -r [tema_origen]:=[tema_destino]

En este caso específico:

.. code:: bash

   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/turtle1/cmd_vel

Esto se entiende como:

   “Cuando este nodo publique en ``/cmd_vel``, en realidad envíalo a
   ``/turtle1/cmd_vel``.”

--------------

¿Por qué es útil el remapeo?

- Evita modificar el código fuente de los nodos.
- Permite **reutilizar nodos genéricos** con distintos robots o
  simuladores.
- Facilita pruebas rápidas con distintos sistemas.

Servicios
---------

Un **servicio** en ROS 2 es una operación remota que sigue un patrón de
**llamada y respuesta** (request-response).

**Componentes de un servicio:**

- **Cliente**: el nodo que inicia la solicitud.
- **Servidor**: el nodo que atiende la solicitud y envía la respuesta.
- **Tipo de servicio**: define la estructura de los datos solicitados y
  respondidos.

--------------

**Estructura de un Servicio**

Los servicios están definidos por archivos ``.srv``, que contienen dos
partes separadas por ``---``:

::

   # Ejemplo: example_interfaces/srv/AddTwoInts

   int64 a
   int64 b
   ---
   int64 sum

Esto significa que: 

- El **cliente** enviará dos enteros ``a`` y ``b`` 

- El **servidor** responderá con un entero ``sum``

--------------

**Comunicación Cliente-Servidor**

La interacción funciona así: |Seleccion| *Tomado de: ROS2 Humble - Documentación Oficial*

::

   [ Cliente ] -- Request --> [ Servidor ]
   [ Cliente ] <-- Response -- [ Servidor ]

- El cliente espera la respuesta antes de continuar.
- Se usa para tareas **puntuales**, **deterministas** y de **control**.

--------------

**¿Cuándo usar un Servicio?**

====================================== ================
Situación                              Usar Servicio
====================================== ================
Encender o apagar un dispositivo       ✅
Calcular una operación matemática      ✅
Consultar el estado de un sistema      ✅
Recibir datos periódicos (sensor, etc) ❌ (usar tópico)
Ejecutar tareas con duración variable  ❌ (usar acción)
====================================== ================

--------------

**Ventajas del uso de servicios**

- Comunicación clara de solicitud y respuesta.
- Ideal para operaciones atómicas.
- Bajo acoplamiento: los nodos solo necesitan conocer el tipo del
  servicio.
- Facilita control de errores: se sabe si hubo respuesta o no.

--------------

**Consideraciones técnicas**

- La espera por la respuesta puede bloquear el nodo (sin ``async``).
- Los servicios **no están diseñados** para flujos continuos de datos.
- Si necesitas emitir múltiples respuestas a una sola solicitud → usa
  **acciones**.

--------------

**Ejemplos de servicios comunes en ROS 2**

+----------------------+---------------------------------------+-----------------------+
| Servicio             | Tipo                                  | Descripción           |
+======================+=======================================+=======================+
| ``/clear``           | ``std_srvs/Empty``                    | Borrar pantalla de    |
|                      |                                       | simuladores como      |
|                      |                                       | turtlesim             |
+----------------------+---------------------------------------+-----------------------+
| ``/reset``           | ``std_srvs/Empty``                    | Reiniciar el estado   |
|                      |                                       | de un nodo            |
+----------------------+---------------------------------------+-----------------------+
| ``/add_two_ints``    | ``example_interfaces/srv/AddTwoInts`` | Sumar dos números     |
+----------------------+---------------------------------------+-----------------------+
| ``/spawn``           | ``turtlesim/srv/Spawn``               | Crear nueva tortuga   |
| (turtlesim)          |                                       |                       |
+----------------------+---------------------------------------+-----------------------+

**Comandos útiles**

Listar servicios disponibles:

.. code:: bash

   ros2 service list

Ver tipo de un servicio:

.. code:: bash

   ros2 service type /nombre_servicio

Ver definición de un servicio:

.. code:: bash

   ros2 interface show [servicio]

Llamar un servicio desde terminal:

.. code:: bash

   ros2 service call /[servicio] "argumentos"

Acciones
--------

**¿Qué es una Acción en ROS 2?**

En ROS 2, una **acción** (``action``) es una estructura de comunicación
que permite la ejecución de tareas **a largo plazo**, donde se necesita:

- Enviar un **objetivo (goal)** desde un nodo cliente a un nodo
  servidor.
- Recibir **retroalimentación (feedback)** mientras se ejecuta la tarea.
- Obtener un **resultado (result)** cuando finaliza la acción.
- Tener la posibilidad de **cancelar la ejecución** en cualquier
  momento.

--------------

**¿Por qué usar acciones y no servicios o tópicos?**

=================== ============== ============= =============
Característica      Tópico         Servicio      Acción
=================== ============== ============= =============
Comunicación        Unidireccional Bidireccional Bidireccional
Tiempo de ejecución Continuo       Corto         Largo
Respuesta directa   ❌             ✅            ✅
Retroalimentación   ❌             ❌            ✅
Cancelación posible ❌             ❌            ✅
=================== ============== ============= =============

En **Conclusión:** Las acciones combinan lo mejor de los servicios y
tópicos, permitiendo tareas de larga duración con control dinámico.

--------------

**Estructura de un archivo ``.action``**

Un archivo ``.action`` define tres partes:

::

   # Objetivo (Goal)
   int32 x
   int32 y
   ---
   # Resultado (Result)
   bool success
   ---
   # Retroalimentación (Feedback)
   float32 percentage_complete

--------------

**Componentes Clave**

- **Cliente de acción:** Nodo que solicita ejecutar una acción.
- **Servidor de acción:** Nodo que ejecuta la acción y responde con
  resultados o feedback.
- **Goal:** Datos de entrada que representan la tarea.
- **Result:** Salida final cuando se completa la acción.
- **Feedback:** Información de progreso enviada mientras se ejecuta.

--------------

**Flujo de Ejecución**

1. El cliente envía un **objetivo (goal)**.
2. El servidor **acepta o rechaza** el objetivo.
3. Si es aceptado, el servidor comienza la ejecución.
4. Se envía **feedback** continuamente.
5. Cuando termina, se envía un **resultado**.
6. El cliente puede **cancelar** la acción si es necesario.´

.. figure:: ./acciones.gif
   :alt: gif

   Tomado de: ROS2 Humble - Documentación Oficial

**Casos de Uso Típicos**

- Navegar a una posición (navegación autónoma).
- Ejecutar trayectorias robóticas.
- Manipular objetos.
- Esperar eventos con tiempo prolongado.
- Controlar movimiento con retroalimentación.

--------------

**Ejemplos de acciones existentes**

- ``nav2_msgs/action/NavigateToPose``
- ``control_msgs/action/FollowJointTrajectory``
- ``moveit_msgs/action/MoveGroup``
- ``lifecycle_msgs/action/Transition``

.. |Seleccion| image:: ./servicios.gif
