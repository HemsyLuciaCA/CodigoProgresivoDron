import cv2
from ultralytics import YOLO
import random
from tracker import Tracker
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Conectar al vehículo (Dron)
vehicle = connect('udp:172.17.168.133:19980', wait_ready=True)

# Parámetros generales
KNOWN_HEIGHT = 1.68  # Altura promedio de una persona (en metros)
FOCAL_LENGTH = 4850  # Longitud focal de la cámara (en píxeles)
GND_SPEED = 0.125  # Velocidad de tierra en m/s
DETECTION_THRESHOLD = 0.5
FRAME_CENTER_OFFSET = 50  # Margen para movimiento en X
FRAME_HEIGHT_BOUNDS = (72, 72)  # Límites para control en Y
DISTANCE_BOUNDS = (50.0, 150.0)  # Distancia deseada al objeto (en cm)
FRAME_DOWNSCALE = 0.5  # Escalado para reducir resolución del frame procesado

# Colores aleatorios para las pistas
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in range(10)]

# Conexión a la cámaraAxel-0402
stream_url = "http://172.17.191.143:8080/stream.mjpeg?clientId=wv98ih2XN8vUQL93"
cap = cv2.VideoCapture(stream_url)
#cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FPS, 5)  # Reducir FPS de 30 a 15


# Modelo YOLO y Tracker
model = YOLO("nano.pt")
tracker = Tracker()

# Función para calcular la distancia a la cámara
def distance_to_camera(known_height, focal_length, per_height):
    return (known_height * focal_length) / per_height

# Función para controlar el dron usando velocidades en los ejes NED
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # BITMASK para considerar solo las velocidades
        0, 0, 0,  # Posición (no cambia)
        vx, vy, vz,  # Velocidades
        0, 0, 0,  # Aceleraciones (no cambiamos)
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Función para controlar el dron según la detección
def control_drone(x_center, y_center, distance):
    if distance < DISTANCE_BOUNDS[0]:
        set_velocity_body(vehicle, -GND_SPEED, 0, 0)  # Retroceder
    elif distance > DISTANCE_BOUNDS[1]:
        set_velocity_body(vehicle, GND_SPEED, 0, 0)  # Avanzar

    if x_center < 240 - FRAME_CENTER_OFFSET:
        set_velocity_body(vehicle, 0, -GND_SPEED, 0)  # Izquierda
    elif x_center > 240 + FRAME_CENTER_OFFSET:
        set_velocity_body(vehicle, 0, GND_SPEED, 0)  # Derecha

    if y_center < 144 - FRAME_HEIGHT_BOUNDS[0]:
        set_velocity_body(vehicle, 0, 0, -GND_SPEED)  # Subir
    elif y_center > 144 + FRAME_HEIGHT_BOUNDS[1]:
        set_velocity_body(vehicle, 0, 0, GND_SPEED)  # Bajar

# Bucle principal
ret, frame = cap.read()
start_time = time.time()
frame_count = 0
frame_skip =30
while ret:
    frame_count += 1
    if frame_count % frame_skip != 0:
        continue
    
    # Reducir resolución para acelerar el procesamiento
    #frame = cv2.resize(frame, None, fx=FRAME_DOWNSCALE, fy=FRAME_DOWNSCALE)

    # Predicción con YOLO
    #results = model.predict(task="detect", source=frame, imgsz=160, max_det=2, conf=0.60, device="cpu", verbose=False, save=False)

    import torch
    with torch.no_grad():
        results = model.predict(
            task="detect",
            source=frame,
            imgsz=160,
            max_det=2,
            conf=0.60,
            device="cpu",
            verbose=False,
            save=False,
            show_labels=False,
            show_conf=False
        )



    # Procesar resultados
    for result in results:
        detections = [
            [int(x1), int(y1), int(x2), int(y2), score]
            for x1, y1, x2, y2, score, class_id in result.boxes.data.tolist()
            if score > DETECTION_THRESHOLD
        ]

        if detections:
            tracker.update(frame, detections)
            for track in tracker.tracks:
                x1, y1, x2, y2 = track.bbox
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                track_id = track.track_id
                per_height = y2 - y1
                distance = distance_to_camera(KNOWN_HEIGHT, FOCAL_LENGTH, per_height)
                x_center, y_center = (x1 + x2) // 2, (y1 + y2) // 2

                control_drone(x_center, y_center, distance)
                color = colors[track_id % len(colors)]
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"ID: {track_id} Dist: {distance:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    cv2.imshow("Tracking", frame)

    # Salir con 'ESC'
    if cv2.waitKey(1) & 0xFF == 27:
        break

    ret, frame = cap.read()

cap.release()
cv2.destroyAllWindows()
vehicle.close()
