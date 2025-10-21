import cv2
import numpy as np

# Dirección del stream UDP
url = "udp://172.17.168.133:19980"  # URL del stream UDP

# Configura el pipeline de GStreamer
gstreamer_pipeline = f"udpsrc address=172.17.168.133 port=19980 ! application/x-rtp, payload=96 ! decodebin ! videoconvert ! video/x-raw, format=BGR ! appsink"

# Abre el stream UDP usando OpenCV y GStreamer
cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
print(cap) # awd

# Verifica que se haya abierto correctamente
if not cap.isOpened():
    print("Error: No se pudo abrir el stream UDP.")
    exit()

# Dimensiones del frame
width, height = 640, 480  # Cambia estos valores según el stream

while True:
    # Lee un frame del stream
    ret, frame = cap.read()

    if not ret:
        print("Error: No se pudo recibir el frame.")
        break

    # Muestra el frame
    cv2.imshow("Frame", frame)

    # Controla el tiempo entre frames (espera por 1 ms) y sale si presionas 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera los recursos
cap.release()
cv2.destroyAllWindows()
