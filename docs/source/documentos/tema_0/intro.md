# ¿Qué es ROS2?

ROS 2 (**Robot Operating System 2**) es la evolución del sistema original ROS. Aporta mejoras clave en seguridad, rendimiento, escalabilidad y soporte para sistemas distribuidos. ROS 2 está diseñado para producción en entornos industriales y robótica avanzada.

Aunque su nombre sugiere que es un sistema operativo, **ROS 2 no es un sistema operativo tradicional**, sino una capa de software que corre sobre uno (como Ubuntu Linux), utilizando el middleware DDS como base para la comunicación entre procesos.

---

## Ecosistema ROS 2

El ecosistema de ROS 2 incluye:

- **Paquetes (packages):** Componentes de software reutilizables que contienen nodos, configuraciones, interfaces, etc.
- **Nodos (nodes):** Programas individuales que se comunican entre sí.
- **Topics:** Canales para mensajes unidireccionales y asíncronos.
- **Services:** Para llamadas sincrónicas tipo request-response.
- **Actions:** Para tareas que pueden tomar tiempo y requieren feedback (como mover un brazo).
- **Launch files (Python):** Permiten iniciar múltiples nodos con configuraciones específicas.
- **Rosbags (ros2 bag):** Graba y reproduce datos de topics para pruebas y análisis.

---

## Arquitectura de ROS 2

ROS 2 está basado en una arquitectura distribuida sobre DDS (**Data Distribution Service**), eliminando la dependencia del nodo maestro (`roscore` ya no existe). Componentes clave incluyen:

- `ros2 run`: Ejecuta un nodo de un paquete
- `ros2 topic`: Inspecciona y publica topics
- `ros2 service`: Llama y describe servicios
- `ros2 action`: Interactúa con acciones
- `ros2 param`: Manejo de parámetros dinámicos
- `ros2 bag`: Grabación y reproducción de datos
- `colcon build`: Compila el workspace (sustituye a catkin_make)

---

## Plumbing & Capabilities

- **Plumbing:** Comunicación de bajo nivel — nodos, topics, services, actions.
- **Capabilities:** Habilidades de más alto nivel construidas sobre lo anterior:
- **Nav2:** Sistema de navegación autónoma para robots móviles
- **SLAM Toolbox:** Localización y mapeo simultáneo
- **MoveIt 2:** Planificación y control de brazos robóticos
- **Percepción:** Procesamiento de imágenes y datos de sensores

---


ROS 2 te permite:

- Desarrollar robots modernos con comunicación robusta y segura
- Reutilizar componentes y colaborar en proyectos a gran escala
- Simular, planificar y controlar hardware real
- Integrarse con microcontroladores, sistemas distribuidos y la nube.

Para una introducción visual a ROS 2 en Español:

🔗 [¿Qué es ROS 2? - Introducción (YouTube)](https://www.youtube.com/watch?v=v3dcy6UEN7g)

