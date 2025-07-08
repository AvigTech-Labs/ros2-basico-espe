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
