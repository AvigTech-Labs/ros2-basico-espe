Ejercicio práctico 2
====================

Actividad
---------

Diseñar y simular un sistema de clasificación de objetos basados en ROS2, utilizando 2 robots Scaras.

Archivos Xacro
--------------

Introducción
~~~~~~~~~~~~

En ROS 2, la descripción de robots se realiza mediante **URDF (Universal Robot Description Format)**.
Sin embargo, a medida que un robot crece en complejidad o cuando se requiere reutilizar el mismo modelo
(varios robots iguales, variantes de hardware, etc.), escribir URDF plano se vuelve poco mantenible.

Para resolver esto, ROS utiliza **XACRO (XML Macros)**, un preprocesador que permite generar URDF de forma
parametrizable, modular y escalable.

Instalación de XACRO
~~~~~~~~~~~~~~~~~~~~

XACRO no siempre viene instalado por defecto.  
Debe instalarse explícitamente para la distribución de ROS 2 en uso.

Instalación:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-$ROS_DISTRO-xacro

Verificación:

.. code-block:: bash

   which xacro
   xacro --version

Si el comando ``xacro`` responde correctamente, la instalación es correcta.

Conceptos generales
~~~~~~~~~~~~~~~~~~~

XACRO es un **preprocesador de URDF**.

Permite:

- Variables (``property``)
- Parámetros de entrada (``arg``)
- Macros reutilizables
- Inclusión de archivos
- Operaciones matemáticas
- Configuraciones condicionales

Analogía útil:

+----------------+----------------+
| Programación   | ROS            |
+================+================+
| Código fuente  | ``.xacro``     |
+----------------+----------------+
| Binario        | ``.urdf``      |
+----------------+----------------+


Estructura básica de un archivo XACRO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Encabezado mínimo:

.. code-block:: xml

   <?xml version="1.0"?>
   <robot xmlns:xacro="http://ros.org/wiki/xacro" name="mi_robot">

     <!-- Contenido -->

   </robot>


Uso de argumentos y propiedades
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Para pasar parámetros desde el launch (por ejemplo, un prefijo), se utilizan ``arg`` y ``property``.
Ejemplo correcto y robusto:

.. code-block:: xml

   <xacro:arg name="prefix" default=""/>
   <xacro:property name="prefix" value="$(arg prefix)"/>

Esto permite usar el prefijo en expresiones:

.. code-block:: xml

   <link name="${prefix}base_link"/>


En ROS 2, no basta solo con ``xacro:arg`` si se va a usar la variable con ``${}``.  
Debe convertirse explícitamente en ``property`` para evitar errores como:

::

   error: name 'prefix' is not defined


Ejemplo de uso de prefijo en links y joints
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: xml

   <link name="${prefix}base_link"/>

   <joint name="${prefix}joint_1" type="revolute">
     <parent link="${prefix}base_link"/>
     <child  link="${prefix}link_1"/>
     <axis xyz="0 0 1"/>
     <limit lower="-3.14" upper="3.14" effort="5" velocity="1.0"/>
   </joint>

Esto garantiza que múltiples instancias del mismo robot no colisionen en TF ni en nombres de joints.

Generación de URDF desde XACRO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Conversión manual:

.. code-block:: bash

   ros2 run xacro xacro robot.urdf.xacro prefix:=r1_ > robot_r1.urdf

En ROS 2, normalmente esta conversión se hace en tiempo de ejecución desde un archivo launch.

Uso de XACRO en archivos launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ejemplo típico en ROS 2:

.. code-block:: python

   robot_description = Command([
       "ros2 run xacro xacro ",
       xacro_file,
       " prefix:=r1_"
   ])

   Node(
       package="robot_state_publisher",
       executable="robot_state_publisher",
       parameters=[{
           "robot_description": robot_description
       }]
   )

Esto permite lanzar el mismo robot múltiples veces con diferentes parámetros.


Multi-robot con XACRO
~~~~~~~~~~~~~~~~~~~~~

Para lanzar dos robots iguales que actúen de forma independiente:

- Un solo archivo XACRO
- Prefijos distintos
- Un ``robot_state_publisher`` por robot
- Un ``joint_states`` por robot
- Transformaciones ``world -> base_link`` independientes

Ejemplo de prefijos:

- Robot 1: ``prefix:=r1_``
- Robot 2: ``prefix:=r2_``

Frames resultantes:

- ``r1_base_link``
- ``r2_base_link``

Esto evita conflictos en:

- TF
- nombres de joints
- visualización en RViz


Dependencias recomendadas en package.xml
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Para un paquete que use XACRO:

.. code-block:: xml

   <exec_depend>xacro</exec_depend>
   <exec_depend>robot_state_publisher</exec_depend>
   <exec_depend>joint_state_publisher_gui</exec_depend>
   <exec_depend>rviz2</exec_depend>
   <exec_depend>tf2_ros</exec_depend>

Errores comunes
~~~~~~~~~~~~~~~

- Ejecutar ``xacro`` sin tenerlo instalado
- Usar ``${prefix}`` sin definir ``property``
- No usar prefijos en sistemas multi-robot
- Compartir ``/joint_states`` entre robots
- Confiar solo en namespaces sin prefijar links/joints

