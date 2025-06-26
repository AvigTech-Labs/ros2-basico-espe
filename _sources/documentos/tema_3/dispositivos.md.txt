# Conexión con dispositivos 

## MQTT

**MQTT (Message Queuing Telemetry Transport)** es un protocolo de mensajería ligero diseñado para dispositivos con recursos limitados y redes de baja calidad. Es ideal para aplicaciones **IoT (Internet de las cosas)** como sensores, actuadores o microcontroladores.

Ventajas de MQTT:
- Bajo consumo de ancho de banda
- Arquitectura **publish/subscribe**
- Soporta calidad de servicio (QoS)
- Uso extendido en la comunidad de microcontroladores (ESP32, ESP8266)

Niveles de QoS

| QoS | Garantía de entrega               | Uso recomendado         |
|-----|----------------------------------|--------------------------|
| 0   | No garantiza entrega             | Sensores no críticos     |
| 1   | Entrega al menos una vez         | Estados importantes      |
| 2   | Entrega exacta una vez (costoso) | Transacciones críticas   |

---

### Instalación

1. Instalación del broker 

El **broker MQTT** es el servidor encargado de gestionar los mensajes que envían y reciben los dispositivos. Para este caso se usará **Mosquitto**, un broker ligero y ampliamente utilizado.

```bash
sudo apt update
sudo apt upgrade
sudo apt install mosquitto mosquitto-clients
```
Esto instalará:
- `mosquitto`: el servicio que actuará como broker
- `mosquitto_pub` y `mosquitto_sub`: herramientas para publicar/suscribirse a tópicos MQTT desde consola


2. Arranque de mosquitto
```bash
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
sudo systemctl status mosquitto
sudo ss -tulnp | grep 1883
```

- `enable mosquitto` habilita el servicio de mosquitto
- `start mosquitto` arranca el servicio de mosquitto
- `status mosquitto` verifica el estado del servicio de mosquitto
- `ss -tulnp | grep 1883` comprobación de los puertos habilitados

3. Instalación del cliente MQTT en Python
```bash
sudo ufw status
pip install paho-mqtt
```
`paho-mqtt` es una biblioteca mantenida por Eclipse que permite conectarse a brokers MQTT desde Python.

4. Pruebas del servidor 

En la terminal 1
```bash
mosquitto_sub -h localhost -t test
```
En la terminal 3
```bash
mosquitto_pub -h localhost -t test -m "Hola mundo"
```

5. Configuración del Broker Mosquitto

Para aceptar conexiones externas (por ejemplo, desde un ESP32), debemos configurar Mosquitto.

***Editar archivo de configuración:***
```bash
sudo nano /etc/mosquitto/conf.d/default.conf
```
Agregar el siguiente contenido:
```
listener 1883
allow_anonymous true
```
- `listener 1883`: habilita el puerto por defecto de MQTT
- `allow_anonymous true`: permite conexiones sin autenticación (ideal para pruebas locales)

***Habilitar el puerto en el firewall de ser necesario***
```bash
sudo ufw allow 1883
```
***Reiniciar Mosquitto***
sudo systemctl restart mosquitto 
sudo systemctl status mosquitto

***Verificar que está escuchando***
```bash
sudo netstat -tulnp | grep 1883
```

***Pruebas de funcionamiento***

En la terminal 1 
```bash
mosquitto_sub -h 192.168.100.176 -t test
```
En la terminal 2
```bash
mosquitto_pub -h 192.168.100.176 -t test -m "ss"
```

### Ejercicios 

Para revisar el funcionamiento de MQTT, simularemos el intercambio de información entre 2 equipos:

Dentro del paquete mi_pkg_python crear el directorio `mqtt`

En el archivo equipo1.py
```python
import paho.mqtt.client as mqtt
import json
import time

# Configuración del broker
BROKER            = "192.168.100.76"
TOPIC_SUB_SEN     = "rm_1/sensores"
TOPIC_SUB_EST     = "rm_1/estados"
TOPIC_PUB         = "rm_1/acciones"
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
        print(f"[{msg.topic}] -> {data}")
        if msg.topic == TOPIC_SUB_SEN:
            print("Mensaje recido del Equipo2:", data["EncoderI"])
        
        if msg.topic == TOPIC_SUB_EST:
            print("Mensaje recido del Equipo2:", data["Estado"])

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
        print("Publicado en datos_1:", payload)
        time.sleep(5)

except KeyboardInterrupt:
    print("\n Finalizando conexión MQTT...")

finally:
    client.loop_stop()
    client.disconnect()
    print(" Desconectado correctamente.")

```
En el archivo equipo2.py
```python

import paho.mqtt.client as mqtt
import json
import time

# Configuración del broker
BROKER            = "192.168.100.76"
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
        print(f"[{msg.topic}] -> {data}")
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
            "Encoderi": 20,
            "Encoderd": 30,
            }
        mensaje = json.dumps(payload)
        client.publish(TOPIC_PUB_SEN, mensaje)
        print("Publicado:", payload)

        payload2 = {
            "Estado": "Encedido"
            }
        mensaje2 = json.dumps(payload2)
        client.publish(TOPIC_PUB_EST, mensaje2)
        print("Publicado:", payload)


        time.sleep(5)

except KeyboardInterrupt:
    print("\n Finalizando conexión MQTT...")

finally:
    client.loop_stop()
    client.disconnect()
    print(" Desconectado correctamente.")
```

---

## Arduino IDE 2.x
Una vez configurado mosquitto, es necesario descargar arduino IDE, el programa que se utilazará para programar la ESP32.

### Instalación

1. ***Descargar Arduino IDE***
Desde: https://www.arduino.cc/en/software/


2. ***Instalar soporte para ESP32***

    1. Ir a `Archivo → Preferencias`
    2. En el campo **URLs adicionales para tarjetas**, agregar:
    ```
    https://espressif.github.io/arduino-esp32/package_esp32_index.json
    ```
    3. Ir a `Herramientas → Placa → Gestor de tarjetas`
    4. Buscar `esp32` e instalar **versión 3.x**

Esta versión es compatible con las últimas bibliotecas MQTT y WiFi.

---

3. Acceso a puertos USB***

El ESP32 se conecta como un dispositivo serie (`/dev/ttyUSB*` o `/dev/ttyACM*`) generalmente la conexión se realiza en el puerto ttyUSB. Para acceder sin sudo:

Añadir usuario a grupo dialout**
```bash
sudo usermod -aG dialout $USER
```

Reiniciar sesión para aplicar cambios.

Ver dispositivos conectados
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Ver cambios al conectar o desconectar
```bash
watch -n 1 ls /dev/ttyUSB* /dev/ttyACM*
```

---

4. Instalación de bibliotecas recomendadas en Arduino IDE

Tabla de librerías recomendadas para tu proyecto ESP32 con WiFiManager y MQTT
|Librería | Autor |	Versión  recomendada|	Notas importantes|
|---------|-------|---------------------|--------------------|
|WiFiManager | tzapu |	2.0.17	|Compatible con ESP32 core 3.x. Asegura el uso del namespace correcto para ESP32. Soporta parámetros personalizados.|
|WiFi	|Espressif|	Incluida en el core	|No requiere instalación manual. Ya viene con el core de ESP32.|
|EEPROM	|Espressif|	Incluida en el core	|Opcional migrar a Preferences si deseas mayor robustez.|
|PubSubClient |	Nick O'Leary	|2.8.0	|Para conexión MQTT (broker IP, topic, publish/subscribe). Ligera y muy estable.|
|ArduinoJson  |Benoit Blanchon	|7.x.x	|Usa DynamicJsonDocument y StaticJsonDocument|

---

### Ejemplos de Uso.

1. Uso de MQTT desde Arduino

Crear 2 pestañas: `Esp32_Mqtt` y `Conf_MQTT`

***Pestaña 1***
```cpp

```


Notas adicionales:
***Publicar mensaje:***
```cpp
client.publish("topic", mensaje_json.c_str(), 1);  // QoS 1
```

***Suscribirse a un tópico:***
```cpp
client.subscribe("Carrito_1/Acciones", 0);  // QoS 0
```