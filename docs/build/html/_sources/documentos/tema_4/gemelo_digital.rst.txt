Gemelo Digital
==============

Archivos Xacro en ROS 2
-----------------------

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
--------------------

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
-----------------------------

URDF
~~~~

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
~~~~~

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
--------------------------

Para el siguiente ejemplo se usa el archivo urdf_2.zip, dentro del directorio mi_pkg_python crearemos
el directorio urdf_2.

.. code-block:: text

   urdf_2
       ├── meshes
       └── urdf_2.urdf

Resultado:

Para convertir un archivo ``.urdf`` a ``.urdf.xacro`` se deben seguir varios
pasos.

1. Cambiar la extensión del archivo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Por ejemplo, si se tiene:

.. code-block:: text

   ensamblaje.urdf

se puede crear una nueva versión:

.. code-block:: text

   touch ensamblaje.urdf.xacro

2. Agregar el espacio de nombres de XACRO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

En un URDF normal se suele tener algo como:

.. code-block:: xml

   <robot name="ensamblaje">

En XACRO se debe cambiar por:

.. code-block:: xml

   <robot xmlns:xacro="http://ros.org/wiki/xacro" name="ensamblaje">

3. Agregar un argumento para el prefijo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

   <mesh filename="package://mi_pkg_python/urdf_2/meshes/base_link.STL"/>

Esto puede mantenerse igual.

Probar el archivo XACRO
-----------------------

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
------------------------

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
---------------------

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
------------------------------

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
         "urdf_2",
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
----------------------

Generación de los robots
~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~

Cada robot se lanza dentro de un namespace diferente:

.. code-block:: python

   namespace="r1"
   namespace="r2"

Esto permite separar tópicos como:

.. code-block:: text

   /r1/joint_states
   /r2/joint_states

Joint states independientes
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Cada robot tiene su propio ``joint_state_publisher_gui``:

.. code-block:: python

   joint_state_publisher_r1
   joint_state_publisher_r2

Esto permite mover cada robot de forma independiente.

Transformaciones fijas al mundo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
----------------------

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
---------------------------

Para que el paquete declare correctamente sus dependencias, se recomienda agregar
en ``package.xml``:

.. code-block:: xml

   <exec_depend>xacro</exec_depend>
   <exec_depend>robot_state_publisher</exec_depend>
   <exec_depend>joint_state_publisher_gui</exec_depend>
   <exec_depend>rviz2</exec_depend>
   <exec_depend>tf2_ros</exec_depend>

Instalación de archivos en setup.py
-----------------------------------

En paquetes tipo ``ament_python``, se debe asegurar que el archivo XACRO se
instale correctamente.

Ejemplo:

.. code-block:: python

   data_files=[
      ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
      ('share/' + package_name + '/urdf_1', ['urdf_1/ensamblaje.urdf']),
      ('share/' + package_name + '/urdf_1/meshes', [
         'urdf_1/meshes/base_link.STL',
         'urdf_1/meshes/brazo_link.STL',
         'urdf_1/meshes/antebrazo_link.STL',
         'urdf_1/meshes/efector_link.STL',
      ]),
      ('share/' + package_name + '/urdf_2', ['urdf_2/ensamblaje.urdf.xacro']),
      ('share/' + package_name + '/urdf_2/meshes', [
         'urdf_2/meshes/base_link.STL',
         'urdf_2/meshes/brazo_link.STL',
         'urdf_2/meshes/antebrazo_link.STL',
         'urdf_2/meshes/efector_link.STL',
      ]),
      ('share/' + package_name + '/launch', [
         'launch/visualizar_rviz.launch.py',
         'launch/visualizar_xacro.launch.py',
         'launch/xacro_robots.launch.py'
         ]),
      ],

Después de modificar ``setup.py`` o agregar archivos nuevos, se debe recompilar:

.. code-block:: bash

   colcon build --symlink-install
   source install/setup.bash

Errores comunes
---------------

Error: xacro no encontrado
~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~

Mensaje típico:

.. code-block:: text

   error: name 'prefix' is not defined

Solución:

.. code-block:: xml

   <xacro:arg name="prefix" default=""/>
   <xacro:property name="prefix" value="$(arg prefix)"/>

Robots que se mueven juntos
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Si al mover un robot se mueven ambos, probablemente los dos están usando el
mismo tópico ``/joint_states``.

Solución:

- Usar namespaces diferentes.
- Remapear ``/joint_states`` a ``joint_states`` dentro de cada namespace.
- Usar un ``joint_state_publisher_gui`` por robot.

Conflictos de TF
~~~~~~~~~~~~~~~~

Si RViz muestra errores de TF o un robot se superpone con otro, probablemente
los links tienen nombres repetidos.

Solución:

- Agregar ``${prefix}`` a todos los links y joints.
- Verificar que existan ``r1_base_link`` y ``r2_base_link``.

Aquitectura ESP32 y Gemelo Digital
----------------------------------

Para realizar una arquitectura compatible con un gemelo digital con ROS2 y la ESP32 es necesario utilizar programas no bloqueantes.

.. tabs::

   .. group-tab:: ESP32_control

      Pestaña 1 del codigo de la ESP32.

      .. code:: cpp

         #include <PubSubClient.h>
         #include <WiFi.h>
         #include <ArduinoJson.h>
         #include <AccelStepper.h>

         // Pines motores paso a paso
         #define STEP2 18
         #define DIR2  19
         #define STEP3 22
         #define DIR3  23

         // Pines Driver
         static const int PIN_STEP = 25;  // -> PUL+
         static const int PIN_DIR  = 27;  // -> DIR+

         // --------------------
         // Config según tu DIP: 400 pulse/rev (del motor)
         // Reductora: PG47  46.656:1
         // --------------------

         static const long PULSES_PER_REV_MOTOR = 400;
         static const float GEAR_RATIO = 46.656f;

         static const long PULSES_PER_REV_OUT = (long)(PULSES_PER_REV_MOTOR * GEAR_RATIO);

         // Convierte grados del eje de salida a pulsos
         long outDegreesToPulses(float deg_out) {
         float pulses = (deg_out / 360.0f) * (float)PULSES_PER_REV_OUT;
         return lroundf(pulses);
         }

         AccelStepper motor1(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
         AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);  // Q1
         AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);  // Q2


         float gradosPorPaso = 0.9;

         // Variable de Control Stepper
         int activar  = 0;
         int estado   = 0;
         int ultra    = 0;
         float q1_mov = 0.0;
         float q2_mov = 0.0;
         float q3_mov = 0.0;

         // Variables de control envio de mensajes
         bool moving = false;
         bool sent_final = false;
         volatile bool nuevoTarget = false;
         long target1;
         int  target2, target3;

         // GPIO de salidad Digital
         int pin_led   = 2;

         //  Credenciales Wifi
         const char* ssid = "CARMEN GONZALEZ_";
         const char* password = "123wa321vg";

         // Credenciales MQTT
         const char* mqtt_broker = "192.168.100.208";
         const int mqtt_port     = 1883;
         const char* cliente     = "rm1_esp32";

         // Temas MQTT Publicar
         const char* tema_sub_juntas  = "ra_1/juntas";
         const char* tema_sub_efector = "ra_1/efector";
         const char* tema_pub_sen     = "ra_1/sensores";

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

         // NEMA 23
         // AccelStepper
         motor1.setMaxSpeed(4665.6);       // pulsos/seg (ojo: a mayor, más exigente)
         motor1.setAcceleration(3110);   // pulsos/seg^2
         motor1.setMinPulseWidth(5);     // us (seguro para DM542T)

         // NEMA 17
         motor2.setMaxSpeed(99.9);
         motor2.setAcceleration(66.7);
         motor2.setPinsInverted(true, false, false); //

         motor3.setMaxSpeed(200);
         motor3.setAcceleration(100);

         // Referencia inicial (si estás arrancando desde "cero")
         motor1.setCurrentPosition(0);
         motor2.setCurrentPosition(0);
         motor3.setCurrentPosition(0);
         }

         void loop() {
         Loop_MQTT();
         if (nuevoTarget) {
            nuevoTarget = false;

            motor1.moveTo(target1);
            motor2.moveTo(target2);
            motor3.moveTo(target3);

            moving = true;
            sent_final = false;
         }

         motor1_q1.run();
         motor1_q2.run();
         motor3.run();

         // actualizacion de grados
         q1_mov_1 = (motor1.currentPosition() * 360.0f) / (float)PULSES_PER_REV_OUT;
         q2_mov = motor2.currentPosition() * gradosPorPaso;
         q3_mov = motor3.currentPosition() * gradosPorPaso;

         // Detectar si ya terminó el movimiento
         bool stillMoving = anyMotorMoving();

         if (moving && !stillMoving && !sent_final) {
         // acaba de terminar
         // Enviar un último paquete final
         estado = 0; // Estado DONE
         envioDatos(tema_pub_sen, estado, ultra, q1_mov, q2_mov, q3_mov);
         sent_final = true;
         moving = false;
         }

         // Publicación periódica cada 100 ms SOLO si está moviendo
         if (moving && (millis() - lastTime >= 100)) {
         estado = 1;  // Estado MOVING
         ultra = ultra + 1;
         envioDatos(tema_pub_sen, estado, ultra, q1_mov, q2_mov, q3_mov);
         
         lastTime = millis();
         }

         if (activar == 1){
            digitalWrite(pin_led, HIGH);  // Enciende el actuador (LED)
            Serial.print("LED ACTIVADO");

            }
         else {
            digitalWrite(pin_led, LOW);  // Apaga el efector (LED)
         }

         }


   .. group-tab:: stepper_control
   
      Pestaña 2 del codigo de la ESP32

      .. code:: cpp

         bool anyMotorMoving() {
         return (motor1.distanceToGo() != 0) ||
                  (motor2.distanceToGo() != 0) ||
                  (motor3.distanceToGo() != 0);
         }
   
   .. group-tab:: stepper_control
   
      Pestaña 3 del codigo de la ESP32

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
               client.subscribe(tema_sub_juntas, 1);
               client.subscribe(tema_sub_efector, 1);
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

         void envioDatos(const char* mqtt_topic_publicar, int estado, int ultra, float q1, float q2, float q3, int robot) {
         DynamicJsonDocument mensaje(256);

         if (mqtt_topic_publicar == tema_pub_sen) {
            
            mensaje["robot"]   = robot;
            mensaje["ultra"]   = ultra;
            mensaje["q1_mov"]  = q1;
            mensaje["q2_mov"]  = q2;
            mensaje["q3_mov"]  = q3;
            mensaje["estado"]   = estado;
            String mensaje_json;
            serializeJson(mensaje, mensaje_json);
            client.publish(mqtt_topic_publicar, mensaje_json.c_str(), false);
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

         Serial.print(topico);

         if (topico == "ra_1/juntas") {
            if (doc.containsKey("q1") && doc.containsKey("q2") && doc.containsKey("q3")) {
               float q1 = doc["q1"]; // NEMA 23
               float q2 = doc["q2"]; // NEMA 17
               float q3 = doc["q3"]; // NEMA 17

               long pasos1 = outDegreesToPulses(q1);
               int pasos2 = round(q2 / gradosPorPaso);
               int pasos3 = round(q3 / gradosPorPaso);

               Serial.print("Recibido q1: ");
               Serial.print(q1);
               Serial.print(" → pasos: ");
               Serial.print(pasos1);


               Serial.print("Recibido q2: ");
               Serial.print(q2);
               Serial.print(" → pasos: ");
               Serial.print(pasos2);

               Serial.print(" Recibido q3: ");
               Serial.print(q3);
               Serial.print(" → pasos: ");
               Serial.println(pasos3);

               target1 = pasos1;
               target2 = pasos2;
               target3 = pasos3;

               nuevoTarget = true;   // <-- SOLO esto

            }
         }

         if (topico == "ra_1/efector") {
            if (doc.containsKey("efector")) {
               activar = doc["efector"];
               Serial.print(" Valor efector recibido");
               Serial.println(activar);
            }

         }

         }

