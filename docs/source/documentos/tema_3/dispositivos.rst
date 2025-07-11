Conexión con dispositivos
=========================

MQTT
----

**MQTT (Message Queuing Telemetry Transport)** es un protocolo de
mensajería ligero diseñado para dispositivos con recursos limitados y
redes de baja calidad. Es ideal para aplicaciones **IoT (Internet de las
cosas)** como sensores, actuadores o microcontroladores.

Ventajas de MQTT: - Bajo consumo de ancho de banda - Arquitectura
**publish/subscribe** - Soporta calidad de servicio (QoS) - Uso
extendido en la comunidad de microcontroladores (ESP32, ESP8266)

Niveles de QoS

=== ================================ ======================
QoS Garantía de entrega              Uso recomendado
=== ================================ ======================
0   No garantiza entrega             Sensores no críticos
1   Entrega al menos una vez         Estados importantes
2   Entrega exacta una vez (costoso) Transacciones críticas
=== ================================ ======================

--------------

Instalación
~~~~~~~~~~~

1. Instalación del broker

El **broker MQTT** es el servidor encargado de gestionar los mensajes
que envían y reciben los dispositivos. Para este caso se usará
**Mosquitto**, un broker ligero y ampliamente utilizado.

.. code:: bash

   sudo apt update
   sudo apt upgrade
   sudo apt install mosquitto mosquitto-clients

Esto instalará: - ``mosquitto``: el servicio que actuará como broker -
``mosquitto_pub`` y ``mosquitto_sub``: herramientas para
publicar/suscribirse a tópicos MQTT desde consola

2. Arranque de mosquitto

.. code:: bash

   sudo systemctl enable mosquitto
   sudo systemctl start mosquitto
   sudo systemctl status mosquitto
   sudo ss -tulnp | grep 1883

- ``enable mosquitto`` habilita el servicio de mosquitto
- ``start mosquitto`` arranca el servicio de mosquitto
- ``status mosquitto`` verifica el estado del servicio de mosquitto
- ``ss -tulnp | grep 1883`` comprobación de los puertos habilitados

3. Instalación del cliente MQTT en Python

.. code:: bash

   sudo ufw status
   pip install paho-mqtt

``paho-mqtt`` es una biblioteca mantenida por Eclipse que permite
conectarse a brokers MQTT desde Python.

4. Pruebas del servidor

En la terminal 1

.. code:: bash

   mosquitto_sub -h localhost -t test

En la terminal 3

.. code:: bash

   mosquitto_pub -h localhost -t test -m "Hola mundo"

5. Configuración del Broker Mosquitto

Para aceptar conexiones externas (por ejemplo, desde un ESP32), debemos
configurar Mosquitto.

**Editar archivo de configuración:**

.. code:: bash

   sudo nano /etc/mosquitto/conf.d/default.conf

Agregar el siguiente contenido:

::

   listener 1883
   allow_anonymous true

- ``listener 1883``: habilita el puerto por defecto de MQTT
- ``allow_anonymous true``: permite conexiones sin autenticación (ideal
  para pruebas locales)

**Habilitar el puerto en el firewall de ser necesario**

.. code:: bash

   sudo ufw allow 1883

**Reiniciar Mosquitto** sudo systemctl restart mosquitto sudo systemctl
status mosquitto

**Verificar que está escuchando**

.. code:: bash

   sudo netstat -tulnp | grep 1883

**Pruebas de funcionamiento**

En la terminal 1

.. code:: bash

   mosquitto_sub -h 192.168.100.178 -t test

En la terminal 2

.. code:: bash

   mosquitto_pub -h 192.168.100.178 -t test -m "ss"

Ejercicios
~~~~~~~~~~

Para revisar el funcionamiento de MQTT, simularemos el intercambio de
información entre 2 equipos:

Dentro del paquete mi_pkg_python crear el directorio ``mqtt``

En el archivo equipo1.py


.. tabs::

   .. group-tab:: equipo1.py

      .. code:: python

         import paho.mqtt.client as mqtt
         import json
         import time

         # Configuración del broker
         BROKER            = "192.168.100.176" # Ip del computador o localhost
         TOPIC_SUB_SEN     = "rm1/sensores"
         TOPIC_SUB_EST     = "rm1/estados"
         TOPIC_PUB         = "rm1/acciones"
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
                     print("Mensaje recido del Equipo2 EncoderI:", data["EncoderI"])
               
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
                     "vel": {
                        "u_meta": 0.34,
                        "w_meta": 0.10,

                     } ,
                     "nombre" :"robot1",
                     "avanzar":0,
                     "soltar_obj":0,
                     "parar":1,
                     }
               mensaje = json.dumps(payload)
               client.publish(TOPIC_PUB, mensaje)
               #print("Publicado en datos_1:", payload)
               time.sleep(1)

         except KeyboardInterrupt:
            print("\n Finalizando conexión MQTT...")

         finally:
            client.loop_stop()
            client.disconnect()
            print(" Desconectado correctamente.")


   .. group-tab:: equipo2.py

      .. code:: python

         import paho.mqtt.client as mqtt
         import json
         import time

         # Configuración del broker
         BROKER            = "192.168.100.176" #Ip del computador o Localhost
         TOPIC_PUB_SEN     = "rm1/sensores"
         TOPIC_PUB_EST     = "rm1/estados"
         TOPIC_SUB         = "rm1/acciones"
         CLIENT_ID         = "cliente_Equipo2"

         # Callback cuando se conecta al broker
         def on_connect(client, userdata, flags, rc, properties=None):
            if rc == 0:
               print("Conectado al broker MQTT")
               client.subscribe(TOPIC_SUB)

            else:
               print(f"Error de conexión: código {rc}")

         # Callback al recibir un mensaje
         def on_message(client, userdata, msg):
            try:
               mensaje = msg.payload.decode("utf-8")
               data = json.loads(mensaje)
               if msg.topic == TOPIC_SUB:
                     print("Mensaje recido del Equipo1:", data["vel"]["u_meta"])

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
                     "EncoderI": 20,
                     "EncoderD": 30,
                     }
               mensaje = json.dumps(payload)
               client.publish(TOPIC_PUB_SEN, mensaje)
               print("Publicado:", payload)

               payload2 = {
                     "Estado": "Encedido"
                     }
               mensaje2 = json.dumps(payload2)
               client.publish(TOPIC_PUB_EST, mensaje2)
               #print("Publicado:", payload)
               time.sleep(1)

         except KeyboardInterrupt:
            print("\n Finalizando conexión MQTT...")

         finally:
            client.loop_stop()
            client.disconnect()
            print(" Desconectado correctamente.")

--------------

Arduino IDE 2.x
---------------

Una vez configurado mosquitto, es necesario descargar arduino IDE, el
programa que se utilazará para programar la ESP32.

.. _instalación-1:

Instalación
~~~~~~~~~~~

1. **Descargar Arduino IDE**

Enlace de descargar, AppImage
`arduino <https://www.arduino.cc/en/software/>`__

En la carpeta donde se descargó el AppImage, cambiar el nombre al
archivo descargado por ``arduino-ide``

.. code:: bash

   sudo apt update
   sudo apt upgrade
   sudo install libfuse2
   sudo chmod +x arduino-ide.AppImage
   ./arduino-ide.AppImage

2. **Instalar soporte para ESP32**

   1. Ir a ``Archivo → Preferencias``
   2. En el campo **URLs adicionales para tarjetas**, agregar:

   ::

      https://espressif.github.io/arduino-esp32/package_esp32_index.json

   3. Ir a ``Herramientas → Placa → Gestor de tarjetas``
   4. Buscar ``esp32`` e instalar **versión 3.x**

Esta versión es compatible con las últimas bibliotecas MQTT y WiFi.

--------------

3. Acceso a puertos USB**\*

El ESP32 se conecta como un dispositivo serie (``/dev/ttyUSB*`` o
``/dev/ttyACM*``) generalmente la conexión se realiza en el puerto
ttyUSB. Para acceder sin sudo:

Añadir usuario a grupo dialout*\*

.. code:: bash

   sudo usermod -aG dialout $USER

Reiniciar sesión para aplicar cambios.

Ver dispositivos conectados

.. code:: bash

   ls /dev/ttyUSB*

Ver cambios al conectar o desconectar

.. code:: bash

   watch -n 1 ls /dev/ttyUSB*

--------------

4. Instalación de bibliotecas recomendadas en Arduino IDE

Tabla de librerías recomendadas para tu proyecto ESP32 con WiFiManager y MQTT

.. list-table::
   :header-rows: 1
   :widths: 15 20 20 60

   * - Librería
     - Autor
     - Versión recomendada
     - Notas importantes
   * - WiFiManager
     - tzapu
     - 2.0.17
     - Compatible con ESP32 core 3.x. Asegura el uso del namespace correcto para ESP32. Soporta parámetros personalizados.
   * - WiFi
     - Espressif
     - Incluida en el core
     - No requiere instalación manual. Ya viene con el core de ESP32.
   * - EEPROM
     - Espressif
     - Incluida en el core
     - Opcional migrar a Preferences si deseas mayor robustez.
   * - PubSubClient
     - Nick O’Leary
     - 2.8.0
     - Para conexión MQTT (broker IP, topic, publish/subscribe). Ligera y muy estable.
   * - ArduinoJson
     - Benoit Blanchon
     - 7.x.x
     - Usa DynamicJsonDocument y StaticJsonDocument.


Ejemplos de Uso.
----------------

1. Uso de MQTT desde Arduino

Crear 2 pestañas: ``Esp32_mqtt`` y ``Conf_mqtt``

.. tabs:: 

   .. group-tab:: Esp32_mqtt

      .. code:: cpp

         #include <PubSubClient.h>
         #include <WiFi.h>
         #include <ArduinoJson.h>

         // Variable de Control Alarmar
         int activar  = 0;
         int estado   = 0;
         int encoderi = 0;
         int encoderd = 0;

         // GPIO de salidad Digital
         int pin_led   = 2;

         //  Credenciales Wifi 
         const char* ssid = "CARMEN GONZALEZ_";
         const char* password = "123wa321vg";

         // Credenciales MQTT
         const char* mqtt_broker = "192.168.100.178";
         const int mqtt_port = 1883;
         const char* cliente = "rm1_esp32";

         // Temas MQTT Publicar
         const char* tema_sub = "rm1/acciones";
         const char* tema_pub_est = "rm1/estados";
         const char* tema_pub_sen = "rm1/sensores";

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
         
         }

         void loop() {
         Loop_MQTT();
         if (millis() - lastTime >= 100){
            estado = 1;
            encoderi = encoderi+1;
            envioDatos(tema_pub_est, estado, encoderi, encoderd);
            envioDatos(tema_pub_sen, estado, encoderi, encoderd);
            lastTime = millis();
         }
         if (activar == 1){
            digitalWrite(pin_led, HIGH);  // Enciende el LED
            
            }
         else {
            digitalWrite(pin_led, LOW);  // Enciende el LED
         }
         
         }

   .. group-tab:: Conf_mqtt

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

         void envioDatos(const char* mqtt_topic_publicar, int estado, int encoderi, int encoderd) {
         DynamicJsonDocument mensaje(256);

         if (mqtt_topic_publicar == tema_pub_est) {
            mensaje["estado"]   = estado;
            String mensaje_json;
            serializeJson(mensaje, mensaje_json);
            client.publish(mqtt_topic_publicar, mensaje_json.c_str(), 1);
         }

         if (mqtt_topic_publicar == tema_pub_sen) {
            mensaje["EncoderI"]   = encoderi;
            mensaje["EncoderD"]   = encoderd;
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
         if (topico == "rm1/acciones") {
               activar  = doc["avanzar"];
         }
         }

Notas adicionales: **Publicar mensaje:**

.. code:: cpp

   client.publish("topic", mensaje_json.c_str(), 1);  // QoS 1

**Suscribirse a un tópico:**

.. code:: cpp

   client.subscribe("Carrito_1/Acciones", 0);  // QoS 0

Puente MQTT - ROS2
------------------

Una vez obtenida una base de MQTT para su uso en microcontroladores y
equipos. Se presenta un nodo que funcionará como puente para la
comunicación entre la ESP32 y ROS2.

.. code:: python

   import rclpy
   from rclpy.node import Node
   from rclpy.duration import Duration
   from std_msgs.msg import Float32, Int32
   import paho.mqtt.client as mqtt
   import json

   class MQTTBridge(Node): 
       def __init__(self):
           super().__init__('mqtt_bridge')

           self.pub = self.create_publisher(Float32, 'sensores', 1)
           self.subscription = self.create_subscription(Int32, 'acciones', self.listener_callback, 10)

           self.last_data = None
           self.active = True  # control de publicación activa
           self.last_mqtt_time = self.get_clock().now()

           self.timer = self.create_timer(0.1, self.publish_sensor_data)       # Publicador (10 Hz)
           self.timer_watchdog = self.create_timer(0.5, self.check_timeout)    # Verificador de tiempo

           self.topic_sub = "rm1/sensores"
           self.topic_pub = "rm1/acciones"
           self.mqtt_client = mqtt.Client()
           self.mqtt_client.on_connect = self.on_connect
           self.mqtt_client.on_message = self.on_message
           self.mqtt_client.connect("192.168.100.178", 1883, 60)
           self.mqtt_client.loop_start()

       def listener_callback(self, msg):
           payload = {
               "vel": {"u_meta": 0.34, "w_meta": 0.10},
               "avanzar": msg.data
           }
           msg_mqtt = json.dumps(payload)
           self.mqtt_client.publish(self.topic_pub, msg_mqtt)

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
                   self.last_data = float(data["EncoderI"])
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

--------------

**Nota**

Existe la posibilidad de utilizar ros2 en microcontroladores, sin
embargo se necesitan versiones de nucleo de tarjetas específicas.

Para revisar mas contenido relacionado se puede revisar los siguientes
links:

- micro-ROS oficial: https://micro.ros.org/
- Repositorio Arduino: https://github.com/micro-ROS/micro_ros_arduino
