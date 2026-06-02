Archivos XACRO en ROS 2
===============================

Introducción
============

En ROS 2, los robots normalmente se describen mediante archivos **URDF**
(*Universal Robot Description Format*). El URDF define la estructura del robot:
links, joints, geometría visual, colisiones e inercias.

Sin embargo, cuando un robot crece en complejidad o se necesita reutilizar el
mismo modelo varias veces, escribir todo directamente en URDF se vuelve poco
práctico.

Para esto se utiliza **XACRO** (*XML Macros*), una herramienta que permite
generar archivos URDF de forma más ordenada, reutilizable y parametrizable.

La idea principal es:

.. code-block:: text

   XACRO  ->  genera  ->  URDF
   humano               ROS 2

Es decir:

- **XACRO** es el archivo que editamos nosotros.
- **URDF** es el resultado final que ROS 2 utiliza.

Instalación de XACRO
====================

Para usar XACRO en ROS 2, primero se debe instalar el paquete correspondiente
a la distribución de ROS 2 utilizada.

.. code-block:: bash

   sudo apt update
   sudo apt install ros-$ROS_DISTRO-xacro

Para verificar la instalación:

.. code-block:: bash

   which xacro
   xacro --version

También se puede probar ejecutando:

.. code-block:: bash

   ros2 run xacro xacro --version

Diferencia entre URDF y XACRO
=============================

URDF
----

Un archivo URDF es un archivo XML plano. Define directamente todos los elementos
del robot.

Ejemplo simple en URDF:

.. code-block:: xml

   <link name="base_link"/>

   <joint name="brazo_joint" type="revolute">
     <parent link="base_link"/>
     <child link="brazo_link"/>
   </joint>

El problema aparece cuando se quiere reutilizar el mismo robot varias veces,
por ejemplo para lanzar dos robots iguales en RViz2. Si ambos robots tienen
links llamados ``base_link`` o joints llamados ``brazo_joint``, los nombres se
repiten y se generan conflictos en TF y en ``joint_states``.

XACRO
-----

XACRO permite agregar variables, argumentos y macros.

Por ejemplo, se puede usar un prefijo:

.. code-block:: xml

   <link name="${prefix}base_link"/>

De esta manera, el mismo robot puede generarse como:

.. code-block:: text

   r1_base_link
   r2_base_link

Esto permite lanzar dos robots iguales sin conflictos de nombres.

Conversión de URDF a XACRO
==========================

Para convertir un archivo ``.urdf`` a ``.urdf.xacro`` se deben seguir varios
pasos.

1. Cambiar la extensión del archivo
-----------------------------------

Por ejemplo, si se tiene:

.. code-block:: text

   ensamblaje.urdf

se puede crear una nueva versión:

.. code-block:: text

   ensamblaje.urdf.xacro

2. Agregar el espacio de nombres de XACRO
-----------------------------------------

En un URDF normal se suele tener algo como:

.. code-block:: xml

   <robot name="ensamblaje">

En XACRO se debe cambiar por:

.. code-block:: xml

   <robot xmlns:xacro="http://ros.org/wiki/xacro" name="ensamblaje">

3. Agregar un argumento para el prefijo
--------------------------------------

Justo después de abrir la etiqueta ``robot``, se agrega:

.. code-block:: xml

   <xacro:arg name="prefix" default=""/>
   <xacro:property name="prefix" value="$(arg prefix)"/>

Estas dos líneas son muy importantes.

La primera línea declara el argumento:

.. code-block:: xml

   <xacro:arg name="prefix" default=""/>

La segunda línea permite usar ese argumento como variable dentro del archivo:

.. code-block:: xml

   <xacro:property name="prefix" value="$(arg prefix)"/>

Si no se agrega la propiedad, puede aparecer el error:

.. code-block:: text

   error: name 'prefix' is not defined

4. Agregar el prefijo a links y joints
--------------------------------------

Antes:

.. code-block:: xml

   <link name="base_link"/>

   <joint name="brazo_joint" type="revolute">
     <parent link="base_link"/>
     <child link="brazo_link"/>
   </joint>

Después:

.. code-block:: xml

   <link name="${prefix}base_link"/>

   <joint name="${prefix}brazo_joint" type="revolute">
     <parent link="${prefix}base_link"/>
     <child link="${prefix}brazo_link"/>
   </joint>

Se debe agregar ``${prefix}`` en:

- ``link name``
- ``joint name``
- ``parent link``
- ``child link``

Ejemplo más completo:

.. code-block:: xml

   <link name="${prefix}base_link"/>

   <link name="${prefix}brazo_link"/>

   <joint name="${prefix}brazo_joint" type="revolute">
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <parent link="${prefix}base_link"/>
     <child link="${prefix}brazo_link"/>
     <axis xyz="0 0 1"/>
     <limit lower="-2.95" upper="2.95" effort="5.0" velocity="1.57"/>
   </joint>

Importante:

Los archivos STL o DAE normalmente no necesitan prefijo, porque son archivos
compartidos por ambos robots.

Ejemplo:

.. code-block:: xml

   <mesh filename="package://mi_pkg_python/urdf/meshes/base_link.STL"/>

Esto puede mantenerse igual.

Probar el archivo XACRO
=======================

Antes de usar el archivo XACRO en un launch, es recomendable probarlo desde la
terminal.

Ejemplo:

.. code-block:: bash

   ros2 run xacro xacro src/mi_pkg_python/urdf/ensamblaje.urdf.xacro prefix:=r1_

También se puede generar un URDF temporal:

.. code-block:: bash

   ros2 run xacro xacro src/mi_pkg_python/urdf/ensamblaje.urdf.xacro prefix:=r1_ > /tmp/robot_r1.urdf

Si no aparece ningún error, el archivo XACRO está correctamente definido.

XACRO en archivos launch
========================

En ROS 2, normalmente no se genera manualmente el archivo URDF. En su lugar,
se ejecuta XACRO desde el archivo launch y el resultado se pasa directamente al
parámetro ``robot_description``.

Ejemplo básico:

.. code-block:: python

   from launch.substitutions import Command
   from launch.substitutions import PathJoinSubstitution
   from launch_ros.substitutions import FindPackageShare

   xacro_file = PathJoinSubstitution([
       FindPackageShare("mi_pkg_python"),
       "urdf",
       "ensamblaje.urdf.xacro"
   ])

   robot_description = Command([
       "ros2 run xacro xacro ",
       xacro_file,
       " prefix:=r1_"
   ])

Luego se entrega esta descripción al nodo ``robot_state_publisher``:

.. code-block:: python

   Node(
       package="robot_state_publisher",
       executable="robot_state_publisher",
       parameters=[{
           "robot_description": robot_description
       }]
   )

El nodo ``robot_state_publisher`` se encarga de publicar las transformaciones TF
del robot a partir del URDF generado y de los datos de ``joint_states``.

Multi-robot con XACRO
=====================

Cuando se desea lanzar dos robots iguales en un solo RViz2, no se deben duplicar
los archivos URDF manualmente. Lo correcto es usar un solo archivo XACRO y
generar dos versiones con prefijos diferentes.

La idea es:

.. code-block:: text

   ensamblaje.urdf.xacro
       ├── prefix:=r1_  ->  robot 1
       └── prefix:=r2_  ->  robot 2

Resultado:

.. code-block:: text

   r1_base_link
   r1_brazo_link
   r1_brazo_joint

   r2_base_link
   r2_brazo_link
   r2_brazo_joint

De esta forma, los nombres de los links, joints y frames TF no se pisan.

Launch para dos robots iguales
==============================

Ejemplo de archivo launch para lanzar dos robots iguales en RViz2:

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.substitutions import Command
   from launch.substitutions import PathJoinSubstitution
   from launch_ros.substitutions import FindPackageShare


   def generate_launch_description():

       xacro_file = PathJoinSubstitution([
           FindPackageShare("mi_pkg_python"),
           "urdf",
           "ensamblaje.urdf.xacro"
       ])

       robot1_description = Command([
           "ros2 run xacro xacro ",
           xacro_file,
           " prefix:=r1_"
       ])

       robot2_description = Command([
           "ros2 run xacro xacro ",
           xacro_file,
           " prefix:=r2_"
       ])

       robot_state_publisher_r1 = Node(
           package="robot_state_publisher",
           executable="robot_state_publisher",
           name="robot_state_publisher",
           namespace="r1",
           output="screen",
           parameters=[
               {"robot_description": robot1_description},
               {"publish_robot_description": True}
           ],
           remappings=[
               ("/joint_states", "joint_states"),
               ("/robot_description", "robot_description")
           ]
       )

       robot_state_publisher_r2 = Node(
           package="robot_state_publisher",
           executable="robot_state_publisher",
           name="robot_state_publisher",
           namespace="r2",
           output="screen",
           parameters=[
               {"robot_description": robot2_description},
               {"publish_robot_description": True}
           ],
           remappings=[
               ("/joint_states", "joint_states"),
               ("/robot_description", "robot_description")
           ]
       )

       joint_state_publisher_r1 = Node(
           package="joint_state_publisher_gui",
           executable="joint_state_publisher_gui",
           name="joint_state_publisher_gui",
           namespace="r1",
           output="screen",
           remappings=[
               ("/joint_states", "joint_states")
           ]
       )

       joint_state_publisher_r2 = Node(
           package="joint_state_publisher_gui",
           executable="joint_state_publisher_gui",
           name="joint_state_publisher_gui",
           namespace="r2",
           output="screen",
           remappings=[
               ("/joint_states", "joint_states")
           ]
       )

       static_tf_r1 = Node(
           package="tf2_ros",
           executable="static_transform_publisher",
           name="static_tf_r1",
           output="screen",
           arguments=[
               "0", "0", "0",
               "0", "0", "0",
               "world",
               "r1_base_link"
           ]
       )

       static_tf_r2 = Node(
           package="tf2_ros",
           executable="static_transform_publisher",
           name="static_tf_r2",
           output="screen",
           arguments=[
               "1.0", "0", "0",
               "0", "0", "0",
               "world",
               "r2_base_link"
           ]
       )

       rviz2 = Node(
           package="rviz2",
           executable="rviz2",
           name="rviz2",
           output="screen"
       )

       return LaunchDescription([
           robot_state_publisher_r1,
           robot_state_publisher_r2,
           joint_state_publisher_r1,
           joint_state_publisher_r2,
           static_tf_r1,
           static_tf_r2,
           rviz2
       ])

Explicación del launch
======================

Generación de los robots
------------------------

Estas líneas generan dos URDF diferentes a partir del mismo XACRO:

.. code-block:: python

   robot1_description = Command([
       "ros2 run xacro xacro ",
       xacro_file,
       " prefix:=r1_"
   ])

   robot2_description = Command([
       "ros2 run xacro xacro ",
       xacro_file,
       " prefix:=r2_"
   ])

El primer robot tendrá links y joints con prefijo ``r1_``.
El segundo robot tendrá links y joints con prefijo ``r2_``.

Namespaces
----------

Cada robot se lanza dentro de un namespace diferente:

.. code-block:: python

   namespace="r1"
   namespace="r2"

Esto permite separar tópicos como:

.. code-block:: text

   /r1/joint_states
   /r2/joint_states

Joint states independientes
---------------------------

Cada robot tiene su propio ``joint_state_publisher_gui``:

.. code-block:: python

   joint_state_publisher_r1
   joint_state_publisher_r2

Esto permite mover cada robot de forma independiente.

Transformaciones fijas al mundo
-------------------------------

Para que ambos robots aparezcan en posiciones diferentes dentro de RViz2, se
publican dos transformaciones fijas:

.. code-block:: text

   world -> r1_base_link
   world -> r2_base_link

Ejemplo:

.. code-block:: python

   arguments=[
       "1.0", "0", "0",
       "0", "0", "0",
       "world",
       "r2_base_link"
   ]

Esto coloca el segundo robot un metro desplazado en el eje X.

Configuración en RViz2
======================

En RViz2:

1. Cambiar ``Fixed Frame`` a:

   .. code-block:: text

      world

2. Agregar un display tipo ``RobotModel`` para el robot 1.

   Description Topic:

   .. code-block:: text

      /r1/robot_description

3. Agregar otro display tipo ``RobotModel`` para el robot 2.

   Description Topic:

   .. code-block:: text

      /r2/robot_description

Si se usa un solo ``RobotModel``, RViz2 puede mostrar solo uno de los robots.
Por eso, para multi-robot, se recomienda agregar un ``RobotModel`` por cada
robot.

Dependencias en package.xml
===========================

Para que el paquete declare correctamente sus dependencias, se recomienda agregar
en ``package.xml``:

.. code-block:: xml

   <exec_depend>xacro</exec_depend>
   <exec_depend>robot_state_publisher</exec_depend>
   <exec_depend>joint_state_publisher_gui</exec_depend>
   <exec_depend>rviz2</exec_depend>
   <exec_depend>tf2_ros</exec_depend>

Instalación de archivos en setup.py
===================================

En paquetes tipo ``ament_python``, se debe asegurar que el archivo XACRO se
instale correctamente.

Ejemplo:

.. code-block:: python

   data_files=[
       ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/urdf', ['urdf/ensamblaje.urdf.xacro']),
       ('share/' + package_name + '/urdf/meshes', [
           'urdf/meshes/base_link.STL',
           'urdf/meshes/brazo_link.STL',
           'urdf/meshes/antebrazo_link.STL',
           'urdf/meshes/efector_link.STL',
       ]),
       ('share/' + package_name + '/launch', [
           'launch/proyecto_rviz2.launch.py'
       ]),
   ]

Después de modificar ``setup.py`` o agregar archivos nuevos, se debe recompilar:

.. code-block:: bash

   colcon build --symlink-install
   source install/setup.bash

Errores comunes
===============

Error: xacro no encontrado
--------------------------

Mensaje típico:

.. code-block:: text

   file not found: [Errno 2] No such file or directory: 'xacro'

Solución:

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-xacro

También se recomienda usar en el launch:

.. code-block:: python

   Command([
       "ros2 run xacro xacro ",
       xacro_file,
       " prefix:=r1_"
   ])

Error: prefix no definido
-------------------------

Mensaje típico:

.. code-block:: text

   error: name 'prefix' is not defined

Solución:

.. code-block:: xml

   <xacro:arg name="prefix" default=""/>
   <xacro:property name="prefix" value="$(arg prefix)"/>

Robots que se mueven juntos
---------------------------

Si al mover un robot se mueven ambos, probablemente los dos están usando el
mismo tópico ``/joint_states``.

Solución:

- Usar namespaces diferentes.
- Remapear ``/joint_states`` a ``joint_states`` dentro de cada namespace.
- Usar un ``joint_state_publisher_gui`` por robot.

Conflictos de TF
----------------

Si RViz muestra errores de TF o un robot se superpone con otro, probablemente
los links tienen nombres repetidos.

Solución:

- Agregar ``${prefix}`` a todos los links y joints.
- Verificar que existan ``r1_base_link`` y ``r2_base_link``.

Buenas prácticas
================

- Usar XACRO en lugar de URDF plano para robots medianos o grandes.
- Usar prefijos para sistemas multi-robot.
- Prefijar links, joints, parent y child.
- Mantener los archivos de mallas compartidos.
- Usar un ``robot_state_publisher`` por robot.
- Usar un ``joint_state_publisher`` o controlador por robot.
- Separar tópicos mediante namespaces.
- Usar ``world`` como frame global en RViz2.
- Probar el XACRO desde terminal antes de lanzarlo con ROS 2.

Conclusión
==========

XACRO permite convertir un modelo URDF rígido en una descripción flexible,
parametrizable y reutilizable.

Para lanzar dos robots iguales en RViz2, la solución recomendada es:

.. code-block:: text

   1 archivo XACRO
   2 prefijos diferentes
   2 robot_state_publisher
   2 joint_state_publisher_gui
   2 transformaciones world -> base_link

La regla principal es:

.. code-block:: text

   XACRO es para escribir modelos reutilizables.
   URDF es lo que ROS 2 finalmente utiliza.
