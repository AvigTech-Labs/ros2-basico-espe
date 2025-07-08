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

1. Nodo suscriptor. Dentro de la carpeta mqtt_python crear el archivo
   ``0_urdf_sub.py``

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

2. Nodo publicador. Dentro de la carpeta mqtt_python crear el archivo
   ``0_urdf_pub.py``

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

--------------

URFD y ESP32
------------

Control de Motor Paso a Paso NEMA 17 con ESP32 y A4988
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Especificaciones Generales

**¿Qué es un paso?**

- Un motor NEMA 17 típico tiene **200 pasos por vuelta**, es decir: (
  1.8° ) por paso en modo paso completo.
- En microstepping, se subdividen los pasos para mayor precisión.

============= ================ ===============
Microstepping Pasos por vuelta Ángulo por paso
============= ================ ===============
Paso completo 200              1.8°
Medio paso    400              0.9°
1/4 paso      800              0.45°
1/8 paso      1600             0.225°
1/16 paso     3200             0.1125°
============= ================ ===============

--------------

2. Configuración de MS1, MS2, MS3 en A4988

============= ==== ==== ====
Resolución    MS1  MS2  MS3
============= ==== ==== ====
Paso completo LOW  LOW  LOW
Medio paso    HIGH LOW  LOW
1/4 paso      LOW  HIGH LOW
1/8 paso      HIGH HIGH LOW
1/16 paso     HIGH HIGH HIGH
============= ==== ==== ====

3. Circuito ESP32 - Motores a paso Nema 17 ESP32 - 38 pines |alt text|
   Circuito |image1|

--------------

4. Alimentación del driver A4988 y Vref

**¿Cómo ajustar el Vref?** 


- Para un NEMA 17 de 1.2 A y Rs = 0.1 Ω:



Si se usa una configuracion a medio paso es recomendado disminuir el
límite de voltaje al 70%.

4. Control por ESP32 - Código Base (medio paso)

.. code:: cpp

   #define STEP_PIN 18
   #define DIR_PIN 19

   void setup() {
     pinMode(STEP_PIN, OUTPUT);
     pinMode(DIR_PIN, OUTPUT);
     digitalWrite(DIR_PIN, HIGH); // Sentido
   }

   void loop() {
     for (int i = 0; i < 400; i++) {
       digitalWrite(STEP_PIN, HIGH);
       delayMicroseconds(1000);
       digitalWrite(STEP_PIN, LOW);
       delayMicroseconds(1000);
     }
     delay(2000);
   }

Control de Motores Paso a Paso con ``AccelStepper`` y ESP32
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**¿Qué es AccelStepper?**

``AccelStepper`` es una librería avanzada para controlar motores paso a
paso de forma más eficiente y profesional. Fue creada para reemplazar el
control básico con ``digitalWrite`` + ``delay()`` con una interfaz más
robusta y no bloqueante.

**Ventajas principales**

- Movimiento **suave** con **aceleración y desaceleración**
- Control **no bloqueante** con ``.run()``
- Soporta **múltiples motores simultáneos**
- Compatible con distintos tipos de driver (por ejemplo, A4988)

--------------

**Parámetros clave**

+-------------------------------+-----------------------------------------------+
| Método                        | Propósito                                     |
+===============================+===============================================+
| ``setMaxSpeed(pasos_s)``      | Establece la velocidad máxima del motor       |
|                               | (pasos/seg)                                   |
+-------------------------------+-----------------------------------------------+
| ``setAcceleration(pasos_s2)`` | Establece la aceleración (pasos/seg²)         |
+-------------------------------+-----------------------------------------------+
| ``move(pasos)``               | Fija un destino relativo en pasos             |
+-------------------------------+-----------------------------------------------+
| ``moveTo(pasos)``             | Fija un destino absoluto en pasos             |
+-------------------------------+-----------------------------------------------+
| ``run()``                     | Ejecuta el movimiento hacia el destino        |
|                               | suavemente                                    |
+-------------------------------+-----------------------------------------------+
| ``isRunning()``               | Devuelve true si el motor sigue en movimiento |
+-------------------------------+-----------------------------------------------+
| ``setCurrentPosition(p)``     | Cambia la posición actual sin mover           |
+-------------------------------+-----------------------------------------------+

--------------

.. code:: cpp

   #include <AccelStepper.h>
   #define STEP_PIN 18
   #define DIR_PIN 19

   // Modo DRIVER usa 1 pulso por paso (la lógica del driver define el microstepping)
   AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

   void setup() {
     Serial.begin(115200);
     delay(1000);
     
     // Configura velocidad y aceleración
     stepper.setMaxSpeed(800);       // pasos por segundo (ajusta según tu driver)
     stepper.setAcceleration(200);   // pasos por segundo^2

     // mov(pasos) movimiento relativo
     // movTo(pasos) movimiento absoluto
     // Función para el Home del robot
   }

   void loop() {
     stepper.moveTo(0);          
     stepper.runToPosition();
     delay(4000);
     // Media Vuelta en sentido horario
     stepper.move(200);
     stepper.runToPosition();
     delay(1000);
     // Media Vuelta en sentido antihorario
     stepper.move(-200);
     stepper.runToPosition();
     delay(1000);
   }

--------------

Gemelo Digital
==============

1. Aplicación básica de movimiento

Utilizando una base del programa mqtt se plantea hacer el control de 2
motores stepper usando la libreria AccelStepper.

.. code:: python

   import paho.mqtt.client as mqtt
   import json
   import time

   # Configuración del broker
   BROKER            = "192.168.100.178" # Ip del computador o localhost
   TOPIC_SUB_SEN     = "ra/sensores"
   TOPIC_SUB_EST     = "ra/estados"
   TOPIC_PUB         = "ra/juntas"
   CLIENT_ID         = "cliente_rm1"

   # Callback cuando se conecta al broker
   def on_connect(client, userdata, flags, rc, properties=None):
       if rc == 0:
           print("Conectado al broker MQTT")
           client.subscribe(TOPIC_SUB_SEN)
           client.subscribe(TOPIC_SUB_EST)

       else:
           print(f"Error de conexión: código {rc}")

   # Callback al recibir un mensaje
   def on_message(client, userdata, msg):
       try:
           mensaje = msg.payload.decode("utf-8")
           data = json.loads(mensaje)
           if msg.topic == TOPIC_SUB_SEN:
               print("Mensaje recido del Equipo2 Ultrasónico:", data["ultra"])
           
           if msg.topic == TOPIC_SUB_EST:
               print("Mensaje recido del Equipo2 Estado:", data["Estado"])

       except Exception as e:
           print("Error procesando mensaje:", e)

   client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv311)

   # Asociar funciones de callback
   client.on_connect = on_connect
   client.on_message = on_message

   # Conexión al broker
   client.connect(BROKER)

   # Iniciar loop en segundo plano
   client.loop_start()

   # Envío continuo de mensajes cada segundo
   try:
       while True:
           payload = {
               "q1": 80,
               "q2": 80,
               "avanzar": 1
               }
           mensaje = json.dumps(payload)
           client.publish(TOPIC_PUB, mensaje)
           #print("Publicado en datos_1:", payload)
           time.sleep(5)

   except KeyboardInterrupt:
       print("\n Finalizando conexión MQTT...")

   finally:
       client.loop_stop()
       client.disconnect()
       print(" Desconectado correctamente.")

Código ESP32

Pestaña 1

.. code:: cpp

   #include <PubSubClient.h>
   #include <WiFi.h>
   #include <ArduinoJson.h>

   #include <AccelStepper.h>

   // Pines motores paso a paso
   #define STEP1 18
   #define DIR1  19
   #define STEP2 22
   #define DIR2  23

   AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);  // Q1
   AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);  // Q2

   float gradosPorPaso = 0.9;

   // Variable de Control Alarmar
   int activar  = 0;
   int estado   = 0;
   int ultra    = 0;

   // GPIO de salidad Digital
   int pin_led   = 2;

   //  Credenciales Wifi 
   const char* ssid = "CARMEN GONZALEZ_";
   const char* password = "123wa321vg";

   // Credenciales MQTT
   const char* mqtt_broker = "192.168.100.178";
   const int mqtt_port     = 1883;
   const char* cliente     = "rm1_esp32";

   // Temas MQTT Publicar
   const char* tema_sub     = "ra/juntas";
   const char* tema_pub_est = "ra/estados";
   const char* tema_pub_sen = "ra/sensores";

   // Variables de control de tiempo
   unsigned long lastTime = 0;

   // Creacion del objeto cliente
   WiFiClient espClient;
   PubSubClient client(espClient);

   // Tamaño de mensaje JSON
   const size_t capacidad_json = JSON_OBJECT_SIZE(30);


   void setup() {
     // Setup Serial
     Serial.begin(115200);
     // Setup Wifi
     setup_wifi();
     // Setup MQTT
     conexion();
     // Manejo del rele
     pinMode(pin_led, OUTPUT);
     motor1.setMaxSpeed(200);     // Ajusta a tus necesidades
     motor1.setAcceleration(100); 
     motor1.setPinsInverted(true, false, false); // 
     motor2.setMaxSpeed(200);
     motor2.setAcceleration(100);
        
   }

   void loop() {
     Loop_MQTT();
     motor1.run();
     motor2.run();

     if (millis() - lastTime >= 100){
       estado = 1;
       ultra = ultra+1;
       envioDatos(tema_pub_est, estado, ultra);
       envioDatos(tema_pub_sen, estado, ultra);
       lastTime = millis();
     }
     if (activar == 1){
       digitalWrite(pin_led, HIGH);  // Enciende el LED
       
       }
     else {
       digitalWrite(pin_led, LOW);  // Enciende el LED
     }
     
   }

Pestaña 2

.. code:: cpp

   void setup_wifi() {
     // Conexión Wifi
     delay(10);
     Serial.println();
     Serial.print("Conectando a ");
     Serial.println(ssid);
     WiFi.begin(ssid, password);
     while (WiFi.status() != WL_CONNECTED) {
       delay(500);
       Serial.print(".");
     }
     Serial.println("");
     Serial.print("WiFi conectado - Dirección IP del ESP: ");
     Serial.println(WiFi.localIP());
   }

   void reconnect() {
     // Control de conexión MQTT
     while (!client.connected()) {
       Serial.print("Intentando conexión MQTT...");
       if (client.connect(cliente)) {
         Serial.println("Conectado");
         // Suscripción a TEMAS
         client.subscribe(tema_sub, 1);
       } else {
         Serial.print("Falló, rc=");
         Serial.print(client.state());
         Serial.println("Intentando de nuevo en 2 segundos");
         delay(2000);
       }
     }
   }

   void conexion() {
     // Conexión MQTT
     client.setServer(mqtt_broker, mqtt_port);
     client.setCallback(callback);
   }

   void Loop_MQTT() {
     // Manejo MQTT 
     if (!client.connected()) {
       reconnect();
     }
     client.loop();
   }

   void envioDatos(const char* mqtt_topic_publicar, int estado, int ultra) {
     DynamicJsonDocument mensaje(256);

     if (mqtt_topic_publicar == tema_pub_est) {
       mensaje["estado"]   = estado;
       String mensaje_json;
       serializeJson(mensaje, mensaje_json);
       client.publish(mqtt_topic_publicar, mensaje_json.c_str(), 1);
     }

     if (mqtt_topic_publicar == tema_pub_sen) {
        mensaje["ultra"]   = ultra;
        String mensaje_json;
       serializeJson(mensaje, mensaje_json);
       client.publish(mqtt_topic_publicar, mensaje_json.c_str(), 1);
     }
     
     
   }

   void callback(char* topic, byte* payload, unsigned int length) {
     StaticJsonDocument<capacidad_json> doc;
     char buffer[length + 1];
     memcpy(buffer, payload, length);
     buffer[length] = '\0';  // Asegura que el buffer tenga fin de cadena

     DeserializationError error = deserializeJson(doc, buffer);

     if (error) {
       Serial.print("Error al deserializar JSON: ");
       Serial.println(error.c_str());
       return;
     }

     String topico(topic);
     if (topico == "ra/juntas") {
       if (doc.containsKey("q1") && doc.containsKey("q2")) {
         float q1 = doc["q1"];
         float q2 = doc["q2"];

         int pasos1 = round(q1 / gradosPorPaso);
         int pasos2 = round(q2 / gradosPorPaso);

         motor1.move(pasos1);
         motor2.move(pasos2);

         Serial.print("Recibido q1: ");
         Serial.print(q1);
         Serial.print(" → pasos: ");
         Serial.println(pasos1);

         Serial.print("Recibido q2: ");
         Serial.print(q2);
         Serial.print(" → pasos: ");
         Serial.println(pasos2);
       }

       if (doc.containsKey("avanzar")) {
         activar = doc["avanzar"];
       }
     }
   }

Con el fin de poder reflejar los movimientos del robot Simulado y del
robot real, se aplica la combinación de los códigos de ``mqtt_bridge`` y
``urdf_sub``.

.. code:: python

   import rclpy
   from rclpy.node import Node
   from rclpy.duration import Duration
   from std_msgs.msg import Float32, Int32
   from sensor_msgs.msg import JointState

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
               JointState, 
               '/joint_states', 
               self.listener_ros, 1
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
           self.mqtt_client.connect("192.168.100.178", 1883, 60)
           self.mqtt_client.loop_start()

       def listener_ros(self, msg):
           q1_mov = 0.0
           q2_mov = 0.0
           q3_mov = 0.0
           for i, name in enumerate(msg.name):
               # Resolucion del stepper 0.9
               if name == "brazo_joint":
                   meta = round(math.degrees(msg.position[i]),4)
                   self.real_q1, q1_mov = self.mover_a_angulo_discreto(meta,self.real_q1)
               if name == "antebrazo_joint":
                   meta = round(math.degrees(msg.position[i]),4)
                   self.real_q2, q2_mov = self.mover_a_angulo_discreto(meta,self.real_q2)

           payload = {
                       "arti_q1": q1_mov,
                       "arti_q2": q2_mov
                     }
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

.. |alt text| image:: image-1.png
.. |image1| image:: circuit_image-1.png


Calibración de Cámara
=====================

Este módulo contiene el proceso completo para la calibración de una cámara utilizando OpenCV, desde la captura de imágenes hasta la corrección de distorsión y detección de AprilTags. Es compatible con ROS 2 y produce un archivo de calibración `camera_calibration.yaml` listo para usarse.

Requisitos
----------
- OpenCV (`pip install opencv-python`)
- PyYAML (`pip install pyyaml`)
- pupil-apriltags (`pip install pupil-apriltags`) para detección de tags

Etapas del proceso
------------------

1. **Captura de Imágenes del Tablero**
   - Archivo: ``0_captura.py``
   - Abre la cámara, muestra una vista previa y guarda imágenes al presionar ``c``.
   - Las imágenes se guardan en la carpeta ``calib_imgs/``.

2. **Calibración con Tablero de Ajedrez**
   - Archivo: ``1_calib.py``
   - Utiliza las imágenes para detectar esquinas de un tablero de 10x7.
   - Genera un archivo ``camera_calibration.yaml`` con la matriz intrínseca y coeficientes de distorsión.

3. **Visualización de Corrección**
   - Archivo: ``3_cam.py``
   - Muestra la imagen original y la corregida en tiempo real utilizando la calibración.

4. **Detección de AprilTags**
   - Archivo: ``4_apriltag.py``
   - Usa ``pupil_apriltags`` para detectar tags y calcular sus poses.
   - Imprime la distancia entre el tag de referencia (ID 0) y los demás.

Parámetros
----------
- Tablero de calibración: 10 x 7 esquinas internas
- Tamaño de cuadrado: 25 mm (0.025 m)
- Distancia y pose se muestran en milímetros
- Salida: `camera_calibration.yaml` (compatible con ROS)

Aplicaciones
------------
- Visión por computadora
- Calibración previa para detección de objetos, SLAM o localización
- Integración con nodos de ROS 2 para transformar coordenadas de cámara



.. tabs::

   .. group-tab:: Programa 1

      .. code-block:: python

         import cv2
         import os

         cap = cv2.VideoCapture(0)
         output_dir = "calib_imgs"
         os.makedirs(output_dir, exist_ok=True)
         count = 0

         cv2.namedWindow("Calibración", cv2.WINDOW_NORMAL)  # Habilita cambio de tamaño
         cv2.resizeWindow("Calibración", 800, 600)           # Establece el tamaño deseado

         while True:
             ret, frame = cap.read()
             if not ret:
                 break
             cv2.imshow("Calibración", frame) 
             key = cv2.waitKey(1) & 0xFF
             if key == ord('c'):  # presiona 'c' para capturar
                 fname = os.path.join(output_dir, f"img_{count:02d}.jpg")
                 cv2.imwrite(fname, frame)
                 print("Imagen guardada:", fname)
                 count += 1
             elif key == ord('q'):  # presiona 'q' para salir
                 break

         cap.release()
         cv2.destroyAllWindows()

   .. group-tab:: Programa 2

      .. code-block:: python

         # Calibración (guardar archivo camera_calibration.yaml)
         import cv2
         import numpy as np
         import os
         import yaml

         # Parámetros del tablero
         CHECKERBOARD = (10, 7)  # esquinas internas: columnas x filas (10x7 → 11x8 cuadrados)
         SQUARE_SIZE = 0.025  # tamaño del cuadrado en metros

         criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

         objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
         objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
         objp *= SQUARE_SIZE

         objpoints = []
         imgpoints = []

         img_dir = "calib_imgs"
         images = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith((".jpg", ".png"))]

         for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                      cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                      cv2.CALIB_CB_NORMALIZE_IMAGE +
                                                      cv2.CALIB_CB_FAST_CHECK)
            if ret:
               objpoints.append(objp)
               corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
               imgpoints.append(corners2)
               cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
               cv2.namedWindow('Corners', cv2.WINDOW_NORMAL)
               cv2.resizeWindow('Corners', 800, 600)
               cv2.imshow('Corners', img)
               cv2.waitKey(500)

         cv2.destroyAllWindows()

         ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

         # Guarda en formato YAML compatible con ROS
         data = {
            'image_width': int(gray.shape[1]),
            'image_height': int(gray.shape[0]),
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': K.flatten().tolist()},
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {'rows': 1, 'cols': len(dist.flatten()), 'data': dist.flatten().tolist()},
         }

         with open('camera_calibration.yaml', 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

         print("Guardado como camera_calibration.yaml")


   .. group-tab:: Programa 3

      .. code-block:: python

         # Archivo para usar la calibración
         import cv2
         import numpy as np
         import yaml

         # Cargar parámetros desde el archivo YAML generado
         with open("camera_calibration.yaml") as f:
            calib_data = yaml.safe_load(f)

         K = np.array(calib_data["camera_matrix"]["data"]).reshape((3, 3))
         dist = np.array(calib_data["distortion_coefficients"]["data"])

         # Captura de cámara
         cap = cv2.VideoCapture(0)

         while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
               break

            h, w = frame.shape[:2]
            new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))

            # Corrección de distorsión
            undistorted = cv2.undistort(frame, K, dist, None, new_camera_mtx)

            # Mostrar ambas imágenes
            cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Undistorted", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Original", 640, 480)
            cv2.resizeWindow("Undistorted", 640, 480)
            cv2.imshow("Original", frame)
            cv2.imshow("Undistorted", undistorted)

            if cv2.waitKey(1) & 0xFF == ord('q'):
               break

         cap.release()
         cv2.destroyAllWindows()



   .. group-tab:: Programa 4

      .. code-block:: python

         import cv2
         import numpy as np
         import yaml
         import time

         # --- Cargar parámetros de calibración desde YAML ---
         with open("camera_calibration.yaml", 'r') as f:
            calib_data = yaml.safe_load(f)

         K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
         dist = np.array(calib_data['distortion_coefficients']['data'])

         # --- Inicializar detección de AprilTags ---
         try:
            from pupil_apriltags import Detector
            at_detector = Detector(families='tag36h11')
         except ImportError:
            raise ImportError("Instala pupil_apriltags con: pip install pupil-apriltags")

         # --- Parámetros del tag ---
         tag_size = 0.08  # Tamaño del tag en metros (100 mm)

         # --- Captura desde la cámara ---
         cap = cv2.VideoCapture(0)
         cv2.namedWindow("AprilTag Detection", cv2.WINDOW_NORMAL)
         cv2.resizeWindow("AprilTag Detection", 800, 600)

         last_print_time = time.time()

         while True:
            ret, frame = cap.read()
            if not ret:
               break

            # Corregir distorsión
            h, w = frame.shape[:2]
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
            undistorted = cv2.undistort(frame, K, dist, None, newcameramtx)

            # Convertir a escala de grises
            gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

            # Detección
            tags = at_detector.detect(
               gray,
               estimate_tag_pose=True,
               camera_params=(K[0, 0], K[1, 1], K[0, 2], K[1, 2]),
               tag_size=tag_size
            )

            tag_poses = {}
            tag_centers = {}

            for tag in tags:
               id = tag.tag_id
               corners = np.int32(tag.corners)
               center = np.mean(corners, axis=0).astype(int)

               cv2.polylines(undistorted, [corners], True, (0, 255, 0), 2)
               cv2.putText(undistorted, f"ID: {id}", tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

               t = tag.pose_t.flatten() * 1000  # Convertir a mm
               tag_poses[id] = t
               tag_centers[id] = tuple(center)

            if 0 in tag_poses:
               ref = tag_poses[0]
               ref_center = tag_centers[0]
               for id, pos in tag_poses.items():
                     if id != 0:
                        rel = pos - ref
                        dist_mm = np.linalg.norm(rel)
                        cv2.putText(undistorted, f"0->{id}: {dist_mm:.1f} mm", (10, 30 + 30 * id),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        if id in tag_centers:
                           cv2.line(undistorted, ref_center, tag_centers[id], (255, 0, 255), 2)

               # Imprimir en terminal cada segundo
               if time.time() - last_print_time >= 1.0:
                     print("\nPosiciones relativas (en mm) con respecto al tag 0:")
                     for id, pos in tag_poses.items():
                        if id != 0:
                           rel = pos - ref
                           print(f"Tag {id}: ΔX={rel[0]:.1f}, ΔY={rel[1]:.1f}, ΔZ={rel[2]:.1f}")
                     last_print_time = time.time()

            cv2.imshow("AprilTag Detection", undistorted)
            if cv2.waitKey(1) & 0xFF == ord('q'):
               break

         cap.release()
         cv2.destroyAllWindows()



Modificacion de archivos.

Publicación de objetos en RVIZ

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from visualization_msgs.msg import Marker
   from geometry_msgs.msg import Point
   from std_msgs.msg import ColorRGBA

   class CuboPublisher(Node):
      def __init__(self):
         super().__init__('cubo_publisher')
         self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
         self.timer = self.create_timer(1.0, self.publicar_cubos)

      def publicar_cubos(self):
         posiciones = [
               (0.1, 0.2, 0.0),
               (0.5, 0.3, 0.0),
               (0.2, 0.3, 0.0)
         ]

         for i, (x, y, z) in enumerate(posiciones):
               cubo = Marker()
               cubo.header.frame_id = 'world'
               cubo.header.stamp = self.get_clock().now().to_msg()
               cubo.ns = 'cubos'
               cubo.id = i
               cubo.type = Marker.CUBE
               cubo.action = Marker.ADD
               cubo.pose.position.x = x
               cubo.pose.position.y = y
               cubo.pose.position.z = z + 0.015  # para que se vea encima del suelo
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
               self.get_logger().info(f'Cubo {i} publicado en ({x}, {y}, {z})')

   def main(args=None):
      rclpy.init(args=args)
      node = CuboPublisher()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()

Actualización del launch visualizar_rviz.

.. code-block:: python

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
               parameters=[{'robot_description': open(urdf_file).read()}]
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
         )
      ])
