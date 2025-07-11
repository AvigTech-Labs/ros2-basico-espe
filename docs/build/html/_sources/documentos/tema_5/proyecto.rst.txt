Ejercicio práctico
==================

Actividad
---------

Diseñar y simular un sistema de clasificación de objetos basados en ROS2.

.. figure:: proyecto.png
   :alt: Imagen

Para los siguientes programas es importante crear un directorio ``proyecto`` dentro del paquete ``mi_pkg_python``. 


Calibración de la cámara
------------------------

Instalación de paquetes
~~~~~~~~~~~~~~~~~~~~~~~

Librerías necesarias:

- OpenCV 

.. code:: bash

   pip install opencv-python

- PyYAML 

.. code:: bash

   pip install pyyaml

- pupil-apriltags para detección de tags

.. code:: bash

   pip install pupil-apriltags

- Codigos de ROS2 desarrollados en este repositorio.

Algoritmo de calibración
~~~~~~~~~~~~~~~~~~~~~~~~

Para calibrar una cámara se utiliza el algoritmo de ajuste de matriz intrínseca basado en un patrón de tablero de ajedrez. Este tipo de calibración permite corregir las distorsiones ópticas generadas por el lente, como la distorsión radial y tangencial, mejorando así la precisión en tareas de visión por computadora.

Los requisitos para realizar la calibración son:

Una estructura fija donde se monte la cámara.

Un tablero de ajedrez de calibración impreso y plano.

Para más detalles sobre el algoritmo utilizado, puedes consultar la documentación oficial de OpenCV:

`opencv- colibración de cámara <https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html>`__

En este ejemplo se utiliza un tablero generado por el proyecto de Mark Hedley Jones, disponible en:

`Tablero de Ajedrez <https://markhedleyjones.com/projects/calibration-checkerboard-collection>`__

Además, se emplean AprilTags de 100 mm para pruebas adicionales de localización.

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

   - Tablero de calibración: 10 x 7 esquinas internas
   - Tamaño de cuadrado: 25 mm (0.025 m)
   - Distancia y pose se muestran en milímetros
   - Salida: `camera_calibration.yaml` (compatible con ROS)


Detección y Transformación de AprilTags
---------------------------------------

**Codigo del proyecto** ``p1_cam.py``

Este nodo en ROS 2 tiene como objetivo detectar etiquetas AprilTags en una imagen, transformar su posición al sistema de referencia del tag con ID 0, y publicar sus coordenadas relativas en el topic ``/apriltag_pixels``.

Estructura general
~~~~~~~~~~~~~~~~~~

El nodo `apriltag_cam` realiza los siguientes pasos:

1. **Carga de parámetros de calibración** desde un archivo YAML.
2. **Inicialización de la cámara y detector AprilTags**.
3. **Corrección de distorsión** de la imagen mediante la matriz de calibración.
4. **Detección de etiquetas** y cálculo de su pose relativa al sistema de la cámara.
5. **Transformación de coordenadas** al sistema de referencia del tag 0.
6. **Publicación** de los datos transformados como mensajes `AprilTagPixelArray`.

Matemática del sistema
~~~~~~~~~~~~~~~~~~~~~~

1. **Calibración de la cámara**

Se utiliza la matriz intrínseca ``K`` y los coeficientes de distorsión ``dist`` para corregir la imagen:

.. code-block:: python

    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, K, dist, None, newcameramtx)

2. **Transformación homogénea**

Cada AprilTag detectado provee:

- Matriz de rotación ``R_tag`` (3x3)
- Vector de traslación ``t_tag`` (3x1)

Estos se combinan para formar una matriz homogénea 4x4:

.. math::

    T = \begin{bmatrix} R & t \\ 0\ 0\ 0 & 1 \end{bmatrix}

3. **Transformación relativa**

Para transformar la posición de cada tag al sistema del tag 0:

.. math::

    T_{\text{rel}} = T_0^{-1} \cdot T_{\text{tag}}

De ahí se extrae la posición relativa:

.. code-block:: python

    T_rel = T0_inv @ T
    t_rel = T_rel[:3, 3] * 1000  # [mm]

Se invierte el eje Y para ROS (orientación hacia arriba):

.. code-block:: python

    msg.posx = float(t_rel[0] / 1000)
    msg.posy = float(-t_rel[1] / 1000)

4. **Orientación del tag 0**

La orientación del tag 0 se expresa en ángulos de Euler para referencia espacial:

.. code-block:: python

    euler = R.from_matrix(R0).as_euler('xyz', degrees=True)

5. **Publicación de mensajes**


Se publica un mensaje tipo ``AprilTagPixelArray`` con múltiples objetos ``AprilTagPixel``, cada uno conteniendo:

- ``id``: ID del tag detectado
- ``posx`` y ``posy``: posición relativa respecto al tag 0 en metros (x hacia adelante, y hacia la izquierda)

6. **Visualización**

La imagen corregida se muestra en una ventana con:

- Contornos de los tags
- Líneas entre tag 0 y los demás
- Texto con distancias relativas


.. note::
   Asegúrese de tener cargado un archivo YAML válido con la calibración de la cámara en la carpeta ``config/`` de su paquete.


Códigos
~~~~~~~

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

   .. group-tab:: p1_cam.py

      .. code-block:: python

         import cv2
         from pupil_apriltags import Detector

         # Transformaciones de la camara
         from scipy.spatial.transform import Rotation as R
         import numpy as np
         import yaml

         # Manejo de ROS2
         import rclpy
         from rclpy.node import Node
         from avig_msg.msg import AprilTagPixel,AprilTagPixelArray

         # Rutas Absolutas del paquete
         from ament_index_python.packages import get_package_share_directory
         import os

         class AprilTagPixelPublisher(Node):
            def __init__(self):
               super().__init__('apriltag_cam')

               self.publisher_data = self.create_publisher(AprilTagPixelArray, '/apriltag_pixels', 1)

               # Obtener ruta absoluta al paquete
               package_path = get_package_share_directory('mi_pkg_python')

               # Ruta completa al archivo de calibración
               yaml_file = os.path.join(package_path, 'config', 'camera_calibration.yaml')

               # Cargar el YAML
               with open(yaml_file, 'r') as f:
                     calib_data = yaml.safe_load(f)

               self.K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
               self.dist = np.array(calib_data['distortion_coefficients']['data'])

               # --- Parámetros del tag ---
               self.tag_size = 0.08  # Tamaño del tag en metros (100 mm)

               self.cap = cv2.VideoCapture(0)

               # Seteo de la ventana de IMSHOW
               self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
               self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
               self.cap.set(cv2.CAP_PROP_FPS, 30)

               if not self.cap.isOpened():
                     self.get_logger().error("No se pudo acceder a la cámara.")
                     exit()

               self.at_detector = Detector(families='tag36h11', nthreads=4)

               # Crear ventana redimensionable (solo si se usa visualización)
               cv2.namedWindow("AprilTag View", cv2.WINDOW_NORMAL)
               cv2.resizeWindow("AprilTag View", 800, 600)

               self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
               self.get_logger().info("Nodo AprilTag CAM iniciado.")

            def timer_callback(self):
               ret, frame = self.cap.read()
               if not ret:
                     self.get_logger().warn(" No se pudo leer el frame.")
                     return
               
               # Corregir distorsión
               h, w = frame.shape[:2]
               newcameramtx, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 1, (w, h))
               undistorted = cv2.undistort(frame, self.K, self.dist, None, newcameramtx)

               # Convertir a escala de grises
               gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

               # Detección
               tags = self.at_detector.detect(
                     gray,
                     estimate_tag_pose=True,
                     camera_params=(self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]),
                     tag_size=self.tag_size
               )

               tag_poses = {}
               tag_centers = {}
               tag_by_id = {}

               for tag in tags:
                     id = tag.tag_id
                     corners = np.int32(tag.corners)
                     center = np.mean(corners, axis=0).astype(int)

                     cv2.polylines(undistorted, [corners], True, (0, 255, 0), 2)
                     cv2.putText(undistorted, f"ID: {id}", tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                     t = tag.pose_t.flatten() * 1000  # Convertir a mm
                     tag_poses[id] = t
                     tag_centers[id] = tuple(center)
                     tag_by_id[id] = tag
               
               msg_arreglo = AprilTagPixelArray()

               # --- Transformación al marco del tag 0 ---

               if 0 in tag_by_id:
                     # Obtener transformación del tag 0
                     ref_tag = tag_by_id[0]
                     R0 = ref_tag.pose_R
                     t0 = ref_tag.pose_t.reshape(3, 1)
                     T0 = np.hstack([R0, t0])
                     T0 = np.vstack([T0, [0, 0, 0, 1]])
                     T0_inv = np.linalg.inv(T0)

                     # Mostrar orientación del tag 0 (ángulos Euler)
                     r_obj = R.from_matrix(R0)
                     euler = r_obj.as_euler('xyz', degrees=True)
                     self.get_logger().info(f'Orientación Tag 0 (Euler): Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}')
                                 
                     # Calcular transformaciones relativas corregidas [mm]
                     
                     for id, tag in tag_by_id.items():
                        if id == 0:
                           msg_0 = AprilTagPixel()
                           msg_0.id = 0
                           msg_0.posx = 0.0
                           msg_0.posy = 0.0
                           continue

                        R_tag = tag.pose_R
                        t_tag = tag.pose_t.reshape(3, 1)
                        T = np.hstack([R_tag, t_tag])
                        T = np.vstack([T, [0, 0, 0, 1]])

                        T_rel = T0_inv @ T
                        t_rel = T_rel[:3, 3] * 1000  # en mm
                        
                        # creacion del tipo de mensaje AprilTagPixel()
                        msg = AprilTagPixel()
                        msg.id = id
                        msg.posx = float(t_rel[0]/1000)
                        msg.posy = float(-t_rel[1]/1000)
                        msg_arreglo.tags.append(msg)

                        if id in tag_centers and 0 in tag_centers:
                           cv2.line(undistorted, tag_centers[0], tag_centers[id], (255, 0, 255), 2)
                           cv2.putText(undistorted, f"0->{id}: {np.linalg.norm(t_rel[:2]):.1f} mm",
                                       (10, 30 + 30 * id), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
               
               self.publisher_data.publish(msg_arreglo)        
               # Visualizacion
               self.visualizar_detecciones(undistorted)

            def visualizar_detecciones(self, frame):
               cv2.imshow("AprilTag View", frame)
               if cv2.waitKey(1) & 0xFF == ord('q'):
                     self.get_logger().info(" 'q' presionado. Saliendo.")
                     self.cap.release()
                     cv2.destroyAllWindows()
                     rclpy.shutdown()

            def destroy_node(self):
               self.cap.release()
               cv2.destroyAllWindows()
               super().destroy_node()

         def main(args=None):
            rclpy.init(args=args)
            node = AprilTagPixelPublisher()
            try:
               rclpy.spin(node)
            except KeyboardInterrupt:
               node.destroy_node()
               rclpy.shutdown()

         if __name__ == '__main__':
            main()


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
