Ejercicio práctico
==================

Actividad
---------

Diseñar y simular un sistema de clasificación de objetos basados en ROS2.



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

`opencv- colibración de cámara < https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html>`__

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
~~~~~~~~~~
- Tablero de calibración: 10 x 7 esquinas internas
- Tamaño de cuadrado: 25 mm (0.025 m)
- Distancia y pose se muestran en milímetros
- Salida: `camera_calibration.yaml` (compatible con ROS)

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

   .. group-tab:: Programa 5

      .. code-block:: python

         print(valor)

Publicación de objetos simulados
--------------------------------

Los objetos que se visualicen a través de la cámara serán digitalizados y simulados en RVIZ2. 

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



Modificacion de archivos launch
-------------------------------

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