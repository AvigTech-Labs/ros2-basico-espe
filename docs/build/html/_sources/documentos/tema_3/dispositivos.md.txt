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
sudo apt install mosquitto mosquitto-clients
```
Esto instalará:
- `mosquitto`: el servicio que actuará como broker
- `mosquitto_pub` y `mosquitto_sub`: herramientas para publicar/suscribirse a tópicos MQTT desde consola

2. Instalación del cliente MQTT en Python
```bash
pip install paho-mqtt
```
`paho-mqtt` es una biblioteca mantenida por Eclipse que permite conectarse a brokers MQTT desde Python.

---

3. Configuración del Broker Mosquitto

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

***Habilitar el puerto en el firewall***
```bash
sudo ufw allow 1883
```

***Reiniciar Mosquitto***
```bash
sudo systemctl restart mosquitto
```

***Verificar que está escuchando***
```bash
sudo netstat -tulnp | grep 1883
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

4. nstalación de bibliotecas recomendadas en Arduino IDE

| Librería       | Propósito                        | Notas |
|----------------|----------------------------------|-------|
| WiFiManager    | Configuración WiFi sin código    | Compatible con ESP32 core 3.x |
| WiFi           | Conexión básica WiFi             | Incluida en el core de ESP32 |
| EEPROM         | Memoria persistente              | Útil para guardar configuración |
| PubSubClient   | Cliente MQTT para microcontroladores | Muy estable y probado |
| ArduinoJson    | Manejo eficiente de JSON         | Ideal para mensajes complejos |

---

## Ejemplos de Uso.


1. Uso de MQTT desde Arduino

### Publicar mensaje:
```cpp
client.publish("topic", mensaje_json.c_str(), 1);  // QoS 1
```

### Suscribirse a un tópico:
```cpp
client.subscribe("Carrito_1/Acciones", 0);  // QoS 0
```