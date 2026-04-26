Ejercicio práctico 2
====================

Actividad
---------

Diseñar y simular un sistema de clasificación de objetos basados en ROS2, utilizando 2 robots Scaras.

Archivos Xacro
--------------

Scaras
------

Máquinas de estado
------------------


Publicación de objetos simulados
--------------------------------

Los objetos que se visualicen a través de la cámara serán digitalizados y simulados en RVIZ2. 

Dependiendo del ID que tenga el tag, se realiza un cambio de color para distinguir la familia de tags relacionada con el tag_id 1 y el tag_id 2.

**Código del proyecto:** ``p1_pub_obj.py``

.. code-block:: python

   import rclpy
   from rclpy.node import Node

   from avig_msg.msg import AprilTagPixelArray
   from visualization_msgs.msg import Marker
   from geometry_msgs.msg import Point
   from std_msgs.msg import ColorRGBA

   # Publicador para el seteo del modelo URDF
   from sensor_msgs.msg import JointState

   class CuboPublisher(Node):
      def __init__(self):
         super().__init__('cubo_publisher')
         self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
         # Suscripcion
         self.subscription = self.create_subscription(
               AprilTagPixelArray,
               '/apriltag_pixels',
               self.listener_callback,
               1)
         self.subscription
         self.msg_cubos = None

         # Timer del publicador
         self.timer = self.create_timer(1.0, self.publicar_cubos)
      
      def listener_callback(self,msg):

         # Llegada del mensaje desde el nodo de la camara
         self.msg_cubos = msg.tags

      def publicar_cubos(self):
         if self.msg_cubos != None:

               # reviso el arreglo del tipo de mensajes apriltags
               for i, tag in enumerate(self.msg_cubos):

                  # area de clasificacion TAG 1
                  if tag.id == 1:
                     cubo = Marker()
                     cubo.header.frame_id = 'world'
                     cubo.header.stamp = self.get_clock().now().to_msg()
                     cubo.ns = 'cubos'
                     cubo.id = i
                     cubo.type = Marker.CUBE
                     cubo.action = Marker.ADD
                     cubo.pose.position.x = tag.posx
                     cubo.pose.position.y = tag.posy
                     cubo.pose.position.z = 0.005  # para que se vea encima del suelo
                     cubo.pose.orientation.x = 0.0
                     cubo.pose.orientation.y = 0.0
                     cubo.pose.orientation.z = 0.0
                     cubo.pose.orientation.w = 1.0
                     cubo.scale.x = 0.10
                     cubo.scale.y = 0.10
                     cubo.scale.z = 0.01
                     cubo.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
                     cubo.lifetime.sec = 0  # 0 = permanente
                     self.publisher.publish(cubo)
                  
                  # area de clasificacion TAG 2

                  if tag.id == 2:
                     cubo = Marker()
                     cubo.header.frame_id = 'world'
                     cubo.header.stamp = self.get_clock().now().to_msg()
                     cubo.ns = 'cubos'
                     cubo.id = i
                     cubo.type = Marker.CUBE
                     cubo.action = Marker.ADD
                     cubo.pose.position.x = tag.posx
                     cubo.pose.position.y = tag.posy
                     cubo.pose.position.z = 0.005  # para que se vea encima del suelo
                     cubo.pose.orientation.x = 0.0
                     cubo.pose.orientation.y = 0.0
                     cubo.pose.orientation.z = 0.0
                     cubo.pose.orientation.w = 1.0
                     cubo.scale.x = 0.10
                     cubo.scale.y = 0.10
                     cubo.scale.z = 0.01
                     cubo.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                     cubo.lifetime.sec = 0  # 0 = permanente
                     self.publisher.publish(cubo)

                  # TAGs  compatibles con el TAG 1
                  if 10 <= tag.id < 20:
                     cubo = Marker()
                     cubo.header.frame_id = 'world'
                     cubo.header.stamp = self.get_clock().now().to_msg()
                     cubo.ns = 'cubos'
                     cubo.id = i
                     cubo.type = Marker.CUBE
                     cubo.action = Marker.ADD
                     cubo.pose.position.x = tag.posx
                     cubo.pose.position.y = tag.posy
                     cubo.pose.position.z = 0.015  # para que se vea encima del suelo
                     cubo.pose.orientation.x = 0.0
                     cubo.pose.orientation.y = 0.0
                     cubo.pose.orientation.z = 0.0
                     cubo.pose.orientation.w = 1.0
                     cubo.scale.x = 0.03
                     cubo.scale.y = 0.03
                     cubo.scale.z = 0.03
                     cubo.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                     cubo.lifetime.sec = 0  # 0 = permanente
                     self.publisher.publish(cubo)
                  
                  # TAGs  compatibles con el TAG 2
                  if 20 <= tag.id < 30:
                     cubo = Marker()
                     cubo.header.frame_id = 'world'
                     cubo.header.stamp = self.get_clock().now().to_msg()
                     cubo.ns = 'cubos'
                     cubo.id = i
                     cubo.type = Marker.CUBE
                     cubo.action = Marker.ADD
                     cubo.pose.position.x = tag.posx
                     cubo.pose.position.y = tag.posy
                     cubo.pose.position.z = 0.015  # para que se vea encima del suelo
                     cubo.pose.orientation.x = 0.0
                     cubo.pose.orientation.y = 0.0
                     cubo.pose.orientation.z = 0.0
                     cubo.pose.orientation.w = 1.0
                     cubo.scale.x = 0.03
                     cubo.scale.y = 0.03
                     cubo.scale.z = 0.03
                     cubo.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                     cubo.lifetime.sec = 0  # 0 = permanente
                     self.publisher.publish(cubo)

                  

   def main(args=None):
      rclpy.init(args=args)
      node = CuboPublisher()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()

Modificación de servicios
-------------------------

Para solucionar el problema de generar una heurística que permita la toma decisiones con respecto a que tag debe clasificar primero el sistema. Se crea un nuevo 
archivo srv ``HeuristicaP`` dentro del paquete ``avig_msg``, teniendo de requermiento y respuesta un ``AprilTagPixelArray``.

Dado que en la sección de tutoriales se creó una solución para este problema, unicamente es necesario cambiar la respuesta del servidor en base al nuevo tipo de
archivo srv

**Código del proyecto** ``p1_heuristica_server.py``

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from avig_msg.srv import HeuristicaP
   from avig_msg.msg import AprilTagPixel
   import math

   class EuristicaServer(Node):
      def __init__(self):

         # Declaracion del nodo
         super().__init__('euristica_server')

         # Creacion del servicio
         self.srv = self.create_service(HeuristicaP, '/heuristica', self.heuristica_callback)
         self.get_logger().info('Servicio Euristica listo.')

      def heuristica_callback(self, request, response):
         tags = request.tags_in.tags
         self.get_logger().info(f"Recibidos {tags} tags.")

         if not tags:
               self.get_logger().warn("No se recibió ningún tag.")
               return response
         
         tag1 = next((tag for tag in tags if tag.id == 1), None)
         tag2 = next((tag for tag in tags if tag.id == 2), None)

         if tag1 is None or tag2 is None:
               self.get_logger().warn("Faltan tag1 o tag2, no se puede continuar.")
               return response


         tags_ordenados = [tag for tag in tags if tag.id != 0 and tag.id != 1 and tag.id != 2]
         
         for i, tag in enumerate(tags_ordenados):
               self.get_logger().info(f"Revisando el tag: {tag.id}")
               if 10 <= tag.id < 20:
                  tag.dist = math.sqrt((tag.posx - tag1.posx)**2 + (tag.posy - tag1.posy)**2)
               else:
                  tag.dist = math.sqrt((tag.posx - tag2.posx)**2 + (tag.posy - tag2.posy)**2)
               
               self.get_logger().info(f"Distancia del Tag: {tag.id} es {tag.dist}")
               
         # Heurística: devolver el tag con menor coordenada posx
         tag_ordenado = sorted(tags_ordenados, key=lambda t: t.dist)
         self.get_logger().info(f"Revisando el tag: {tag_ordenado}")
         response.tags_out.tags = tag_ordenado
         return response

   def main(args=None):
      rclpy.init(args=args)
      node = EuristicaServer()
      rclpy.spin(node)
      rclpy.shutdown()

   if __name__ == '__main__':
      main()

Cinemática Inversa
------------------

Modelo Geométrico
~~~~~~~~~~~~~~~~~

Se considera un manipulador plano con dos eslabones de longitud:

- :math:`l_1`: longitud del brazo
- :math:`l_2`: longitud del antebrazo

Y dos articulaciones:

- :math:`q_1`: ángulo del primer eslabón respecto al eje X de ``base_link``
- :math:`q_2`: ángulo del segundo eslabón respecto al primero

El extremo del robot (end-effector) está ubicado en el plano XY, y el origen del robot está desplazado desde el origen global (``world``) por:

- :math:`dx`: desplazamiento en X
- :math:`dy`: desplazamiento en Y

Formulación
~~~~~~~~~~~

Dado un punto deseado en coordenadas globales:

.. math::

    (x_a, y_a)

El primer paso es trasladar ese punto al marco del robot (``base_link``):

.. math::

    x_d = x_a - dx \\
    y_d = y_a - dy

A partir de este punto deseado relativo al marco base, se aplica la cinemática inversa para resolver los ángulos de las articulaciones.

Se define:

.. math::

    D = \frac{x_d^2 + y_d^2 - l_1^2 - l_2^2}{2 l_1 l_2}

El ángulo :math:`q_2` se obtiene mediante:

.. math::

    q_2 = \arccos(D)

Y el ángulo :math:`q_1` se obtiene por:

.. math::

    q_1 = \arctan2(y_d, x_d) - \arctan2(l_2 \sin(q_2), l_1 + l_2 \cos(q_2))

Restricciones
~~~~~~~~~~~~~

- El valor absoluto de :math:`D` debe ser menor o igual a 1 para garantizar solución real.
- Si :math:`|D| > 1`, el punto está fuera del alcance del robot.

Ejemplo en Código
~~~~~~~~~~~~~~~~~

Con el fin de automatizar el proceso de carga del desplazamiento del origen del URDF del robot SCARA creado, con respecto al april-tag 0, y de los eslabones correspondientes a q1 y q2,
se implementa un algoritmo que a través del uso de TF2 carga automaticamente estos valores al lanzarse el launcher con el URDF y visualizador RVIZ. 

**Código del proyecto** ``p1_ci.py``

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import TransformStamped, Pose
   from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException
   from math import acos, atan2, sqrt
   import numpy as np

   class CinematicaInversaTF(Node):
      def __init__(self):
         super().__init__('cinematica_inversa_tf')
         self.tf_buffer = Buffer()
         self.tf_listener = TransformListener(self.tf_buffer, self)

         self.pub = self.create_publisher(Pose, '/angulos_mov', 1)
         self.sub = self.create_subscription(Pose, '/puntos_ci', self.callback_ci, 1)

         self.timer = self.create_timer(1.0, self.obtener_parametros_robot)

         self.dx = None
         self.dy = None
         self.l1 = None
         self.l2 = None

      def obtener_parametros_robot(self):
         try:
               t0 = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())
               t1 = self.tf_buffer.lookup_transform('brazo_link', 'antebrazo_link', rclpy.time.Time())
               t2 = self.tf_buffer.lookup_transform('antebrazo_link', 'efector_link', rclpy.time.Time())

               self.dx = t0.transform.translation.x
               self.dy = t0.transform.translation.y

               self.l1 = sqrt(t1.transform.translation.x**2 + t1.transform.translation.y**2)
               self.l2 = sqrt(t2.transform.translation.x**2 + t2.transform.translation.y**2)

               self.get_logger().info(f"Obtenido: l1={self.l1:.3f}, l2={self.l2:.3f}, dx={self.dx:.3f}, dy={self.dy:.3f}")
               self.timer.cancel()  # Ya no es necesario repetir

         except (LookupException, TimeoutException) as e:
               self.get_logger().warn("Esperando transformaciones...")

      def callback_ci(self, msg):
         if None in (self.dx, self.dy, self.l1, self.l2):
               self.get_logger().warn("Parámetros del robot aún no disponibles.")
               return

         try:
               xa = msg.position.x
               ya = msg.position.y

               # Convertir a sistema base_link
               xd = xa - self.dx
               yd = ya - self.dy

               D = (xd**2 + yd**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
               if abs(D) > 1:
                  self.get_logger().error("Punto fuera del alcance geométrico.")
                  return

               q2 = acos(D)
               q1 = atan2(yd, xd) - atan2(self.l2 * np.sin(q2), self.l1 + self.l2 * np.cos(q2))

               self.get_logger().info(f"q1 = {q1:.3f} rad, q2 = {q2:.3f} rad")

               msg_enviar = Pose()
               msg_enviar.orientation.x = q1
               msg_enviar.orientation.y = q2
               msg_enviar.position.z = msg.position.z  

               self.get_logger().info(f'Enviando angulos {msg_enviar}')
               self.pub.publish(msg_enviar)

         except Exception as e:
               self.get_logger().error(f"Error en cinemática inversa: {str(e)}")

   def main(args=None):
      rclpy.init(args=args)
      node = CinematicaInversaTF()
      try:
         rclpy.spin(node)
      except KeyboardInterrupt:
         pass
      node.destroy_node()
      rclpy.shutdown()


Conexión ROS2 - ESP32
---------------------

Utilizando el archivo base de urdf-mqtt.py, se realizan las modificaciones necesarias para suscribirse a un topic, que publique los angulos que debe moverse cada juntas
para alcanzar los diferentes objetos en el espacio de trabajo.

**Código del proyecto** ``p1_puente.py``

.. code-block:: python 

   import rclpy
   from rclpy.node import Node
   from rclpy.duration import Duration

   # librerias soporte URDF
   from std_msgs.msg import Float32 
   from sensor_msgs.msg import JointState
   from geometry_msgs.msg import Pose


   # librerias MQTT
   import paho.mqtt.client as mqtt
   import json

   import math

   class MQTTBridge(Node): 
      def __init__(self):
         super().__init__('mqtt_bridge')

         # Angulos del robot
         self.real_q1 = 0.0
         self.real_q2 = 0.0
         self.real_q3 = 0.0

         self.pub = self.create_publisher( Float32, 'sensor_bateria', 1)
         self.subscription = self.create_subscription(
               Pose, 
               '/angulos_mov', 
               self.listener_ros, 1
               )
         
         self.pub_joint = self.create_publisher(
               JointState, 
               '/joint_states', 
               1
               )
         

         self.last_data = None
         self.active = True  # control de publicación activa
         self.last_mqtt_time = self.get_clock().now()

         self.timer = self.create_timer(0.1, self.publish_sensor_data)       # Publicador (10 Hz)
         self.timer_watchdog = self.create_timer(0.5, self.check_timeout)    # Verificador de tiempo

         self.topic_sub = "ra/sensores"
         self.topic_pub = "ra/juntas"
         self.mqtt_client = mqtt.Client()
         self.mqtt_client.on_connect = self.on_connect
         self.mqtt_client.on_message = self.on_message
         self.mqtt_client.connect("192.168.100.180", 1883, 60)
         self.mqtt_client.loop_start()

      def listener_ros(self, msg):

         self.get_logger().info('Arrancando Puente')
         
         q1_mov = 0.0
         q2_mov = 0.0
         q3_mov = 0.0

         # Resolucion del stepper 0.9
         meta = round(math.degrees(msg.orientation.x),4)
         self.real_q1, q1_mov = self.mover_a_angulo_discreto(meta,self.real_q1)

         meta = round(math.degrees(msg.orientation.y),4)
         self.real_q2, q2_mov = self.mover_a_angulo_discreto(meta,self.real_q2)

         payload = {
                     "q1": q1_mov,
                     "q2": q2_mov
                     }
         
         msg_j = JointState()
         msg_j.header.stamp = self.get_clock().now().to_msg()
         msg_j.name = ['brazo_joint', 'antebrazo_joint', 'efector_joint']
         msg_j.position = [math.radians(self.real_q1), math.radians(self.real_q2), msg.position.z]
         self.pub_joint.publish(msg_j)

         self.get_logger().info('Postura inicial publicada.')
         msg_mqtt = json.dumps(payload)
         self.mqtt_client.publish(self.topic_pub, msg_mqtt)
         print("Mensaje Enviado")
      
      def mover_a_angulo_discreto(self, angulo_objetivo, angulo_actual, paso=0.9):
         """
         Calcula el desplazamiento al múltiplo de 'paso' más cercano al ángulo objetivo.
         Retorna:
               - el nuevo ángulo corregido (múltiplo de paso)
               - el desplazamiento angular necesario desde el ángulo actual
         """

         # Calcula desplazamiento
         desplazamiento = angulo_objetivo - angulo_actual

         # Redondea el ángulo objetivo al múltiplo más cercano
         desplazamiento_valido = round(desplazamiento / paso) * paso

         meta_ajustada = angulo_actual + desplazamiento_valido
         print(f"[INFO] Objetivo :{angulo_objetivo}°")
         print(f"[INFO] Objetivo ajustado: {meta_ajustada}° (múltiplo de {paso}°)")
         print(f"[INFO] Desplazamiento desde actual: {desplazamiento_valido:.2f}°")

         return round(meta_ajustada,2), round(desplazamiento_valido,2)


      def on_connect(self, client, userdata, flags, rc):
         if rc == 0:
               print("Conectado al broker MQTT")
               client.subscribe(self.topic_sub)
         else:
               print(f"Error de conexión: código {rc}")

      def on_message(self, client, userdata, msg):
         try:
               mensaje = msg.payload.decode("utf-8")
               data = json.loads(mensaje)
               if msg.topic == self.topic_sub:
                  self.last_data = float(data["bateria"])
                  self.last_mqtt_time = self.get_clock().now()  # Actualiza tiempo del último dato
                  self.active = True
         except Exception as e:
               print("Error procesando mensaje:", e)

      def publish_sensor_data(self):
         if self.last_data is not None and self.active:
               ros_msg = Float32()
               ros_msg.data = self.last_data
               self.pub.publish(ros_msg)
               self.get_logger().info(f"ROS2 publicó: {ros_msg.data}")

      def check_timeout(self):
         now = self.get_clock().now()
         if now - self.last_mqtt_time > Duration(seconds=2.0):
               if self.active:
                  self.get_logger().warn("No se reciben datos desde MQTT. Se detiene la publicación.")
               self.active = False

   def main(args=None):
      rclpy.init(args=args)
      node = MQTTBridge()
      try:
         rclpy.spin(node)
      except KeyboardInterrupt:
         pass
      rclpy.shutdown()

Modificacion de archivos launch
-------------------------------

En el archivo launcher es necesario eliminar el slider creado para la manipulación del URDF, dado a que este ahora se moverá en base al 
algoritmo de control del sistema creado.

**Código del proyecto** ``p1.launch.py``

.. code-block:: python

   # Importa la clase principal para definir lanzamientos en ROS 2
   from launch import LaunchDescription

   # Importa la acción Node para lanzar nodos ROS 2
   from launch_ros.actions import Node

   # Permite obtener la ruta del directorio share de un paquete instalado
   from ament_index_python.packages import get_package_share_directory

   # Controlador de lanzamiento
   from launch.actions import TimerAction

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
               parameters=[{'robot_description': open(urdf_file).read()}]
         ),

         # Nodo que lanza RViz2 para visualizar el robot
         Node(
               package='rviz2',
               executable='rviz2',
               name='rviz2',
               output='screen'
         ),

         # Nodo que agrega una transformación estática: world → base_link
         Node(
               package='tf2_ros',
               executable='static_transform_publisher',
               name='static_tf_pub',
               arguments=['0.10', '0.10', '0.0',  # x y z (en metros)
                        '0', '0', '0',        # roll pitch yaw (en radianes)
                        'world', 'base_link'], # parent frame, child frame
               output='screen'
         ),

         # Nodo personalizados
         Node(
               package='mi_pkg_python',
               executable='p1_pub_obj',
               name='nodo_publicador_objetos',
               output='screen'
         ),

         Node(
               package='mi_pkg_python',
               executable='p1_heur',
               name='nodo_heuristica_server',
               output='screen'
         ),

         Node(
               package='mi_pkg_python',
               executable='p1_ci',
               name='nodo_ci',
               output='screen'
         ),

         Node(
               package='mi_pkg_python',
               executable='p1_puente',
               name='nodo_puente_mqtt',
               output='screen'
         ),

         TimerAction(
               period=2.0,  # esperar 2 segundos
               actions=[
                  Node(
                     package='mi_pkg_python',
                     executable='p1_set',
                     name='init_joints',
                     output='screen'
                  )
               ]
         )
         
      ])


Dado que, se han eliminado los slider es necesario crear un archivo que publique los valores iniciales de las juntas q1,q2 y q3.

**Código del proyecto** ``p1_set.py``

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState

   class JointInitializer(Node):
      def __init__(self):
         super().__init__('init_joints')
         self.publisher = self.create_publisher(JointState, '/joint_states', 1)

         # Publicar al iniciar
         self.timer = self.create_timer(0.5, self.publicar_posicion)

      def publicar_posicion(self):
         msg = JointState()
         msg.header.stamp = self.get_clock().now().to_msg()
         msg.name = ['brazo_joint', 'antebrazo_joint', 'efector_joint']
         msg.position = [0.0, 0.0, 0.0]
         self.publisher.publish(msg)
         self.get_logger().info('Postura inicial publicada.')
         rclpy.shutdown()

   def main(args=None):
      rclpy.init(args=args)
      node = JointInitializer()
      rclpy.spin(node)

   if __name__ == '__main__':
      main()


Algoritmo de control de proceso
-------------------------------

Para realizar el ejercicio de clasificación de objetos es necesario aplicar un algoritmo que permita serilizar el proceso que 
tendra que hacer el robot.

**Código del proyecto** ``p1_coordinador.py``

.. code-block:: python 

   import rclpy
   from rclpy.node import Node
   from rclpy.callback_groups import ReentrantCallbackGroup

   from avig_msg.msg import AprilTagPixelArray
   from avig_msg.srv import HeuristicaP
   from sensor_msgs.msg import JointState
   from geometry_msgs.msg import Pose

   import time

   class NodoCoordinador(Node):
      def __init__(self):
         super().__init__('nodo_coordinador')

         # Tags puntos de almacenes

         self.tag_id_1 = None
         self.tag_id_2 = None
         # Publicador para setear la posición inicial
         self.pub_joint = self.create_publisher(JointState, '/joint_states', 1)
         # CI / Puente
         self.pub_tra = self.create_publisher(Pose, '/puntos_ci', 1)
         # Cliente de servicio y acción
         self.cli = self.create_client(HeuristicaP, '/heuristica')
         self.sub = self.create_subscription(
               AprilTagPixelArray,
               '/apriltag_pixels',
               self.callback_tags,
               1
         )

         self.ultima_data    = None
         self.proceso_activo = False

         self.get_logger().info('P "y" para comenzar el proceso...')
         self.esperar_input_usuario()

      def esperar_input_usuario(self):
         import threading

         def esperar_tecla():
               while True:
                  tecla = input()
                  if tecla.lower() == 'y':
                     self.get_logger().info('Iniciando el proceso...')
                     self.proceso_activo = True
                     break

         threading.Thread(target=esperar_tecla, daemon=True).start()

      def setear_posicion_inicial(self):
         msg = JointState()
         msg.header.stamp = self.get_clock().now().to_msg()
         msg.name = ['brazo_joint', 'antebrazo_joint', 'efector_joint']
         msg.position = [0.0, 0.0, 0.0]
         self.pub_joint.publish(msg)

         ## Sleep de 2 segundos
         self.get_logger().info('Posición inicial seteada.')

      def callback_tags(self, msg):
         if not self.proceso_activo:
               return

         tags = msg.tags
         self.tag_id_1 = next((tag for tag in tags if tag.id == 1), None)
         self.tag_id_2 = next((tag for tag in tags if tag.id == 2), None)

         if self.tag_id_1 is None or self.tag_id_2 is None:
               self.get_logger().warn("Faltan tag1 o tag2, no se puede continuar.")
               self.esperar_input_usuario()
               self.proceso_activo = False
               return
         
         if not self.cli.service_is_ready():
               self.get_logger().warn('Servicio no está disponible.')
               return

         if not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().error('Timeout esperando servicio.')
               return

         request = HeuristicaP.Request()
         request.tags_in.tags = msg.tags
         self.proceso_activo = False
         
         future = self.cli.call_async(request)
         future.add_done_callback(self.llamar_accion)

      def llamar_accion(self, future):
         try:
               self.get_logger().info(f'Solocitando CI')
               response = future.result()

               # Envio de resultados el movimiento del primer valor

               for i, tag in enumerate(response.tags_out.tags):
                  goal_msg = Pose()
                  goal_msg.position.x = tag.posx
                  goal_msg.position.y = tag.posy
                  goal_msg.position.z = 0.0
                  self.pub_tra.publish(goal_msg)
                  time.sleep(3)
                  self.get_logger().info(f'Enviando objetivos al nodo CI 1 {goal_msg}')
                  
                  if 10 <= tag.id < 20:
                     goal_msg = Pose()
                     goal_msg.position.x = self.tag_id_1.posx
                     goal_msg.position.y = self.tag_id_1.posy
                     goal_msg.position.z = 0.03
                     self.pub_tra.publish(goal_msg)
                     time.sleep(3)
                     self.get_logger().info(f'Enviando objetivos al nodo CI 1 {goal_msg}')

               self.get_logger().info(f'Ejercicio completo{goal_msg}')
               rclpy.shutdown()
               
         except Exception as e:
               self.get_logger().error(f'Error en servicio Herustica: {str(e)}')



   def main(args=None):
      rclpy.init(args=args)
      node = NodoCoordinador()
      try:
         rclpy.spin(node)
      except KeyboardInterrupt:
         pass
      rclpy.shutdown()

   if __name__ == '__main__':
      main()


=========================================
Edge Computing + ROS 2 + SLAM 3D
=========================================

Objetivo
========

Este documento resume una arquitectura recomendada (nivel profesional/industrial) para integrar:

- **Edge computing** (cómputo en el robot)
- **ROS 2** como middleware
- **SLAM 3D** (mapeo y localización en 3D)
- Navegación y control (opcional, pero común en robots móviles)

La idea principal es diseñar un sistema robusto que soporte:

- Alto ancho de banda de sensores (LiDAR/Depth)
- Cómputo pesado (odometría + SLAM)
- Baja latencia en control
- Escalabilidad (monitorización, logging y gemelo digital)

--------------------------------------------------------------------

1. ¿Qué se ejecuta en el Edge y qué se ejecuta fuera?
=====================================================

En el Edge (robot)
------------------

En el robot se debe ejecutar todo lo que es **crítico para operación en tiempo real**:

- Drivers de sensores: LiDAR, cámara depth, IMU, encoders
- Sincronización y estampado temporal (timestamps coherentes)
- Preprocesamiento de datos (filtros, downsample, deskew)
- Odometría (wheel/visual/LiDAR)
- **SLAM 3D** (pose + mapa, loop closure si aplica)
- Navegación local (evitación reactiva / control local)
- Control (``ros2_control`` / drivers CAN / PWM / Ethernet)

Fuera del Edge (estación/base/servidor)
---------------------------------------

Fuera del robot se deja lo que no es crítico o puede ser pesado:

- Visualización (RViz2, dashboards web)
- Grabación masiva de datos (rosbag continuo a NAS)
- Analítica y mantenimiento predictivo
- Map merging (multi-robot) / optimizaciones batch
- Gemelo digital / simulación (Isaac Sim, etc.)

Regla de oro
------------

**Todo lo que requiere respuesta inmediata debe vivir en el Edge.**

--------------------------------------------------------------------

2. Requisitos de hardware para SLAM 3D (por clases)
===================================================

El hardware depende principalmente del sensor y del stack SLAM elegido.

A) SLAM 3D basado en LiDAR (robusto en industria)
-------------------------------------------------

- En general exige **CPU fuerte** (procesamiento geométrico y registro)
- GPU opcional dependiendo del pipeline
- RAM moderada/alta (mapa y buffers)

Recomendación típica:

- CPU: 6–12 cores con buen rendimiento por núcleo
- RAM: 16–32 GB
- SSD NVMe: 256 GB–1 TB (mapas + rosbags)
- Red: 1 GbE mínimo (2.5/10 GbE si hay múltiples sensores o alta tasa)

B) SLAM 3D basado en cámara depth / estéreo (visual / RGB-D)
------------------------------------------------------------

- **GPU ayuda mucho** (especialmente con pipelines acelerados)
- Sensible a iluminación y texturas
- Requiere buena sincronización IMU–cámara

Recomendación típica:

- CPU: 6–12 cores
- GPU: NVIDIA (si se busca aceleración / Isaac ROS)
- RAM: 16–32 GB
- SSD NVMe: recomendado

C) SLAM 3D + Percepción avanzada + Navegación completa
------------------------------------------------------

Cuando se ejecuta SLAM + percepción + detección + navegación:

- CPU fuerte + GPU potente + RAM alta
- Recomendado usar mini-PC industrial o plataforma NVIDIA con NVMe

--------------------------------------------------------------------

3. Arquitectura ROS 2 recomendada (bloques)
===========================================

Bloques mínimos (pipeline estándar)
-----------------------------------

1. **Sensor Layer**
   - Drivers: LiDAR / Depth / IMU / encoders

2. **Time & Sync Layer**
   - timestamps consistentes
   - TF bien definido (``map``, ``odom``, ``base_link``)

3. **Preprocessing**
   - VoxelGrid / downsample
   - Crop ROI (recortar volumen útil)
   - Deskew (LiDAR + IMU) si aplica
   - (Opcional) quitar plano (mesa/suelo) y filtrar ruido

4. **Odometry**
   - Wheel + IMU con EKF
   - Visual odom o LiDAR odom según el caso

5. **SLAM 3D**
   - Estimación de pose global
   - Pose-graph / loop closure (según implementación)
   - Mapa 3D (nubes/voxels)

6. **Navigation (si robot móvil)**
   - Planner global + control local (Nav2)
   - Evitación de obstáculos (2D/3D)

7. **Control**
   - ``ros2_control`` y hardware interfaces

Recomendación industrial
------------------------

- Separar por contenedores (Docker) o usar composable nodes por rendimiento
- Usar QoS correcto según tipo de dato (sensores vs control)
- Aislar CPU y prioridades si se requiere determinismo

--------------------------------------------------------------------

4. SLAM 3D en ROS 2 (opciones prácticas)
=======================================

La elección depende del sensor y del objetivo.

Opción 1: RTAB-Map (RGB-D / integración flexible)
-------------------------------------------------

- Muy usado en robótica aplicada
- Loop closure y mapeo
- Puede producir mapa 2D a partir de 3D para navegación (según configuración)

Ideal si:
- Se busca un stack flexible “todo en uno”
- Se usa cámara depth y un odom decente

Opción 2: SLAM basado en LiDAR (robusto para industria)
-------------------------------------------------------

Para entornos con poca luz, polvo o grandes espacios, LiDAR suele ser mejor.
En ROS 2, la selección concreta varía por hardware y disponibilidad.

Ideal si:
- Entorno industrial
- Necesidad de robustez en condiciones complejas

Opción 3: Isaac ROS (si la plataforma es NVIDIA)
------------------------------------------------

- Pipeline acelerado en GPU para percepción y odometría visual (según módulos)
- Muy eficiente en FPS y latencia con cámaras

Ideal si:
- Edge basado en NVIDIA
- Se busca alto rendimiento con visión

--------------------------------------------------------------------

5. Nav2 + SLAM 3D: cómo se integran en la práctica
==================================================

Aunque SLAM sea 3D, en robot móvil la navegación normalmente usa costmaps 2D.

Enfoque típico:

- SLAM 3D produce pose y mapa (3D)
- Se genera o mantiene un **mapa 2D** (ocupación) para Nav2
- Obstáculos dinámicos se actualizan desde:
  - LiDAR/Depth proyectados a 2D
  - voxel layers (según configuración)

Recomendación práctica
----------------------

Mantener:

- **Mapa 2D** para planificación global (estable y eficiente)
- **Percepción 3D local** para obstáculos dinámicos y seguridad

--------------------------------------------------------------------

6. Puntos críticos que suelen fallar en SLAM 3D (y mitigación)
=============================================================

1) Tiempo mal sincronizado
--------------------------

Síntomas:
- Drift elevado
- mapas “gelatinosos”
- desalineación de nubes

Mitigación:
- Asegurar timestamps coherentes
- Verificar sincronización IMU–cámara–LiDAR
- Revisar latencias y buffers

2) TF inconsistente
-------------------

Mitigación:
- Definir claramente frames: ``map``, ``odom``, ``base_link``, ``sensor_link``
- Evitar transforms duplicadas o contradictorias

3) Exceso de datos (saturación de CPU/red/memoria)
--------------------------------------------------

Mitigación:
- VoxelGrid / downsample
- Crop ROI
- Ajustar tasas de publicación (Hz)
- Filtrar ruido y puntos innecesarios

4) QoS incorrecto en ROS 2
--------------------------

Mitigación:
- Sensores: típicamente ``best_effort`` (según caso)
- Control y estado: ``reliable``
- Ajustar profundidad de colas y durabilidad según uso

--------------------------------------------------------------------

7. Arquitectura recomendada (resumen “industrial”)
==================================================

Edge (robot)
------------

- Drivers (LiDAR/Depth/IMU/encoders)
- TF + sincronización
- EKF (wheel + IMU)
- Preprocesamiento de nubes
- Odometry
- **SLAM 3D**
- Nav2 (control local / evitación) si aplica
- ``ros2_control`` / drivers motores

Base station / servidor
-----------------------

- RViz2 / dashboards
- logging y análisis
- gestión de mapas
- gemelo digital y simulación (opcional)

--------------------------------------------------------------------

Conclusión
==========

La combinación **Edge + ROS 2 + SLAM 3D** se diseña como un sistema distribuido:

- El Edge ejecuta lo crítico (sensores, odom, SLAM, control)
- El servidor ejecuta visualización, análisis y funciones pesadas no críticas

Una implementación profesional depende de:

- Hardware apropiado para el sensor principal (LiDAR vs cámara depth)
- TF y tiempo correctamente sincronizados
- Preprocesamiento para evitar saturación
- QoS correcto en ROS 2
- Integración de SLAM 3D con navegación (usualmente costmap 2D + percepción 3D local)
