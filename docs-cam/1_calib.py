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

