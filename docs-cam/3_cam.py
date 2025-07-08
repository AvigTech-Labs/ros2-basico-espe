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

