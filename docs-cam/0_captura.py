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
