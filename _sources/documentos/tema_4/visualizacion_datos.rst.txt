Archivos URDF
=============

Prueba de actualizacion

Para crear archivos URDF en base a diseños personalidados de robots con
solidworks, es necesario descargar la extensión URDF.

`Extension-Solid <https://github.com/ros/solidworks_urdf_exporter>`__,
para versiones de SolidWorks superiores a la 2021. Seleccionar la
opción: SolidWorks 2021

Una vez generado el archivo CAD con el que se va a trabajar, es
necesario configurar el arbol de juntas y eslabones del robot usando la
extensión de archivos URDF.

.. figure:: solid.png
   :alt: Imagen

   Imagen

Esta extensión generará una carpeta con los documentos necesarios para
la simulación el robot. Se puede visualizar la configuración obtenida
utilizando la herramienta online
`viewer-urdf <https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html>`__,
para usarlo, se debe arrastrar la carpeta generada por SolidWorks en la
pantalla de visualizador y esta generará una interfaz de manipulación
rapida del modelo obtenido.

.. figure:: image.png
   :alt: Ejemplo

   Ejemplo

En el siguiente enlace
`Scara-URDF <https://drive.google.com/open?id=15o6Q_H6R-In0UsSA8FMnYaNj3OwC1M9L&usp=drive_fs>`__,
se encuentra el modelo base que será utilizado para configurar el
visualizador RVIZ2 de ROS2.

Visualización de archivos URDF
------------------------------

Para visualizar y manipular el archivo URDF creado se utiliza la
herramienta RVIZ, a continuación se muestra los pasos para su
configuración.

1. Instalar paquetes necesarios

.. code:: bash

   sudo apt install ros-humble-joint-state-publisher-gui

--------------

2. Copiar archivos al paquete

Dentro del paquete ``mi_pkg_python``, se debe crear los directorios:
``urdf/meshes`` y ``launch``.

- Directorio urdf: agregar el archivo urdf generado y dentro del
  subdirectorio meshes: colocar todos los archvos STL
- Directorio launch: crear un archivo vacio
  ``visualizar_rviz.launch.py``

::

   mi_pkg_python/
   ├── urdf/
   │   ├── ensamblaje.urdf
   │   └── meshes/
   │       ├── base_link.STL
   │       └── brazo_link.STL
   │       └── antebrazo_link.STL
   │       └── efector_link.STL
   ├── launch/
   │   ├── visualizar_rviz.launch.py

nota: Es importante revisar la correcta escritura de los diferentes
archivos.

--------------

3. Verifica y edita las rutas en el URDF

Dentro de ``ensamblaje.urdf``, es necesario revisar que las rutas a las
mallas estén definidas con prefijo ``package://``, por ejemplo:

.. code:: xml

   <mesh filename="package://mi_pkg_python/urdf/meshes/base_link.STL"/>

En este punto es necesario revisar que los parámetros sean diferentes de
0, por ejemplo:

.. code:: xml

   <limit lower="-1.57" upper="1.57" effort="1.0" velocity="1.0" />

--------------

4. Modificar ``setup.py`` para instalar los recursos

Dentro de ``setup.py``, realizar la modificación del bloque
``data_files`` de la sigueinte forma:

.. code:: python

   data_files=[
       ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/urdf', ['urdf/ensamblaje.urdf']),
       ('share/' + package_name + '/urdf/meshes', [
           'urdf/meshes/base_link.STL',
           'urdf/meshes/brazo_link.STL',
           'urdf/meshes/antebrazo_link.STL',
           'urdf/meshes/efector_link.STL',
       ]),
       ('share/' + package_name + '/launch', ['launch/visualizar_rviz.launch.py']),
       ],

--------------

5. Crear el archivo de lanzamiento

Crea el archivo ``launch/visualizar_rviz.launch.py`` con el siguiente
contenido:

.. code:: python

   # Importa la clase principal para definir lanzamientos en ROS 2
   from launch import LaunchDescription

   # Importa la acción Node para lanzar nodos ROS 2
   from launch_ros.actions import Node

   # Permite obtener la ruta del directorio share de un paquete instalado
   from ament_index_python.packages import get_package_share_directory

   # Módulo estándar para trabajar con rutas de archivos
   import os

   # Función principal requerida por ROS 2 para ejecutar este archivo de lanzamiento
   def generate_launch_description():
       # Construye la ruta completa del archivo URDF dentro del paquete
       urdf_file = os.path.join(
           get_package_share_directory('mi_pkg_python'),  # Paquete que contiene el URDF
           'urdf',
           'ensamblaje.urdf'
       )

       # Devuelve la lista de nodos a lanzar
       return LaunchDescription([

           # Nodo que publica el URDF en el topic /robot_description
           Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               parameters=[{'robot_description': open(urdf_file).read()}]  # Carga el contenido del URDF
           ),

           # Nodo que abre una interfaz gráfica con sliders para mover las juntas
           Node(
               package='joint_state_publisher_gui',
               executable='joint_state_publisher_gui',
               name='joint_state_publisher_gui',
               output='screen'
           ),

           # Nodo que lanza RViz2 para visualizar el robot
           Node(
               package='rviz2',
               executable='rviz2',
               name='rviz2',
               output='screen'
           )
       ])

**¿Por qué se usan estos nodos?**

- ``robot_state_publisher``: publica la descripción del robot
  (``robot_description``) para que RViz2 la use.
- ``joint_state_publisher_gui``: permite mover las juntas manualmente
  mediante sliders.
- ``rviz2``: lanza la visualización.

--------------

6. Compilar y lanzar

.. code:: bash

   cd ~/ros2_ws
   colcon build --packages-select mi_pkg_python
   ros2 launch mi_pkg_python visualizar_rviz.launch.py

Manejo de datos en RVIZ con URDF.
---------------------------------

En este caso se busca crear archivos que permitan obtener y colocar
valores en las juntas del modelo URDF creado.

Los datos de las juntas se publican y escuchan utilizando el tipo de
mensaje ``JointState``

JointState
~~~~~~~~~~

El mensaje ``JointState`` representa el **estado actual de las
articulaciones** del robot.

**Campos del mensaje**

::

   Header header
   string[] name
   float64[] position
   float64[] velocity
   float64[] effort

+--------------+------------------------------------------------------------+
| Campo        | Descripción                                                |
+==============+============================================================+
| ``name``     | Lista de nombres de juntas (ej:                            |
|              | ``['brazo_joint', 'antebrazo_joint']``)                    |
+--------------+------------------------------------------------------------+
| ``position`` | Posiciones angulares o lineales actuales                   |
+--------------+------------------------------------------------------------+
| ``velocity`` | (Opcional) Velocidad de cada junta                         |
+--------------+------------------------------------------------------------+
| ``effort``   | (Opcional) Torque o esfuerzo de cada junta                 |
+--------------+------------------------------------------------------------+

..

   Solo ``name`` y ``position`` son requeridos para publicar el estado
   del robot.

**Ejemplo:**

Dentro de la carpeta mqtt_python crear los archivos  ``0_urdf_sub.py`` y ``0_urdf_pub.py``

.. tabs::

   .. group-tab:: 0_urdf_sub.py

      .. code:: python

         import rclpy
         from rclpy.node import Node
         from sensor_msgs.msg import JointState

         class JointMonitor(Node):
            def __init__(self):
               super().__init__('joint_monitor')
               self.subscription = self.create_subscription(
                     JointState,
                     '/joint_states',
                     self.callback,
                     1
               )

            def callback(self, msg):
               for i, name in enumerate(msg.name):
                     if name == "brazo_joint" or name == "antebrazo_joint" or name == "efector_joint":
                        self.get_logger().info(f"{name} → {msg.position[i]:.3f} rad")

         def main(args=None):
            rclpy.init(args=args)
            node = JointMonitor()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()

   .. group-tab:: 0_urdf_pub

      .. code:: python

         import rclpy
         from rclpy.node import Node
         from sensor_msgs.msg import JointState
         import math
         import time

         class JointPublisher(Node):
            def __init__(self):
               super().__init__('joint_publisher_manual')
               self.pub = self.create_publisher(JointState, '/joint_states', 1)
               self.timer = self.create_timer(0.1, self.publish_joints)  # 10 Hz
               self.angulo    = 0.0
               self.distancia = 0.0
               self.signo = 1

            def publish_joints(self):
               msg = JointState()
               msg.header.stamp = self.get_clock().now().to_msg()
               msg.name = ['brazo_joint', 'antebrazo_joint', 'efector_joint']
               
               # Movimiento sinusoidal para prueba
               brazo     = math.sin(self.angulo) 
               antebrazo = math.cos(self.angulo) 
               efector   = self.distancia
               
               msg.position = [brazo, antebrazo, efector]

               self.pub.publish(msg)
               self.get_logger().info(f'Publicando: brazo={brazo:.2f}, antebrazo={antebrazo:.2f}, efector {efector:.2f}')

               if self.distancia >= 0.040:
                     self.signo = -1
               
               if self.distancia <= 0.000:
                     self.signo = 1

               self.distancia += 0.001*self.signo 
               self.angulo += 0.005

         def main(args=None):
            rclpy.init(args=args)
            node = JointPublisher()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()

Agregar los programas en el setup.py para poder ejecutarlos con ROS2.

--------------

tf2
~~~

| ``tf2`` es el sistema que gestiona **transformaciones espaciales entre
  los frames** del robot.
| Permite conocer **la posición y orientación de cada parte** del robot
  en tiempo real.

ROS 2 Humble ya incluye tf2 por defecto en la mayoría de instalaciones,
pero se puede instalar instalarlo con:

.. code:: bash

   sudo apt update
   sudo apt install ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-tools

**¿Qué hace?**

- Administra un árbol de transformaciones
- Calcula la pose de cada ``link`` con respecto a otro (``base_link``,
  ``efector_link``, etc.)
- Usa transformaciones 3D (posición + orientación con cuaterniones)

--------------

**¿Quién publica los ``tf``?**

+---------------------------+---------------------------------------------+
| Fuente                    | Transformaciones que publica                |
+===========================+=============================================+
| ``robot_state_publisher`` | Desde ``base_link`` hasta cada ``joint``    |
|                           | (usando URDF + ``/joint_states``)           |
+---------------------------+---------------------------------------------+
| SLAM / navegación         | ``map → odom``, ``odom → base_link``        |
+---------------------------+---------------------------------------------+
| Sensores / cámaras        | ``base_link → sensor_link``, etc.           |
+---------------------------+---------------------------------------------+
| Nodos personalizados      | Transforms adicionales según necesidad      |
+---------------------------+---------------------------------------------+

--------------

**¿Cómo obtener la pose del efector?**

Con ``tf2``, puedes usar:

.. code:: bash

   ros2 run tf2_ros tf2_echo base_link efector_link

**Ejemplo**

1. Dentro de la carpeta mqtt_python crear el archivo ``0_tf2.py``

.. code:: python


   # Importa las clases necesarias de tf2 para escuchar transformaciones
   from tf2_ros import TransformListener, Buffer

   # Importa la API principal de ROS 2 en Python
   import rclpy
   from rclpy.node import Node

   # Define una clase que extiende de Node para crear un nodo ROS 2
   class EffectorPose(Node):
       def __init__(self):
           # Inicializa el nodo con el nombre 'effector_pose'
           super().__init__('effector_pose')

           # Crea un buffer donde se almacenan las transformaciones
           self.tf_buffer = Buffer()

           # Crea un listener que va llenando el buffer automáticamente con los datos de /tf
           self.tf_listener = TransformListener(self.tf_buffer, self)

           # Crea un temporizador que llama a la función lookup_pose cada 0.5 segundos
           self.timer = self.create_timer(0.5, self.lookup_pose)

       # Esta función se llama periódicamente para obtener la pose del efector
       def lookup_pose(self):
           try:
               # Busca la transformación desde 'base_link' hasta 'efector_link'
               # El tiempo es "now", es decir, la transformación más reciente disponible
               t = self.tf_buffer.lookup_transform(
                   'base_link',        # Frame de referencia (padre)
                   'efector_link',     # Frame hijo (efector final)
                   rclpy.time.Time())  # Tiempo: ahora

               # Extrae la posición y orientación de la transformación
               pos = t.transform.translation
               rot = t.transform.rotation

               # Imprime por consola (logger) la pose actual del efector
               self.get_logger().info(
                   f"Pose: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f} | "
                   f"q=({rot.x:.2f}, {rot.y:.2f}, {rot.z:.2f}, {rot.w:.2f})"
               )

           except Exception as e:
               # Si ocurre algún error (ej. aún no existe la transformación), lo muestra como advertencia
               self.get_logger().warn(f"No transform: {e}")

   # Función principal que lanza el nodo
   def main(args=None):
       rclpy.init(args=args)          # Inicializa el sistema ROS 2
       node = EffectorPose()          # Crea una instancia del nodo
       rclpy.spin(node)               # Mantiene el nodo activo procesando eventos
       rclpy.shutdown()              

Agregar el codigo en el setup.py para poder ejercutarlo con ROS2.

**Nota**

Si existe un problema de dependencias al ejecutar el código, edita el
archivo package.xml y asegúrate de incluir:

::

   <exec_depend>tf2_ros</exec_depend>
   <exec_depend>geometry_msgs</exec_depend>
   <exec_depend>builtin_interfaces</exec_depend>


.. |alt text| image:: image-1.png
.. |image1| image:: circuit_image-1.png