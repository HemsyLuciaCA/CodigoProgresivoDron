import cv2
from ultralytics import YOLO
import random
from tracker import Tracker
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,Command,LocationGlobal
from pymavlink import mavutil
import tkinter as tk
# Conectar al vehículo (Dron)
vehicle = connect('udp:192.168.143.28:19980', wait_ready=True)

# Función para calcular la distancia
def distance_to_camera(knownHeight, focalLength, perHeight):
    return (knownHeight * focalLength) / perHeight
# Velocidad de tierra en metros por segundo
gnd_speed = 0.125
vehicle.parameters['RTL_ALT'] = 0

# Función para controlar el dron usando velocidades en los ejes NED (North, East, Down)
def set_velocity_body(vehicle, vx, vy, vz):
    """Controla el dron utilizando las velocidades en los ejes NED (North, East, Down)."""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # BITMASK para considerar solo las velocidades
        0, 0, 0,  # Posición (no cambia)
        vx, vy, vz,  # Velocidades
        0, 0, 0,  # Aceleraciones (no cambiamos)
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Función para controlar el dron
def control_drone(x_center, y_center, distance):
    """
    Controla el dron según la posición y la distancia de la persona detectada.
    """
    # Parámetros de control
    min_distance = 50.0  # Distancia mínima deseada (en centimetros)
    max_distance = 150.0  # Distancia máxima deseada (en centimetros)

    frame_center = 640  # Asumimos que la imagen tiene 1280 píxeles de ancho

    # Controlar la distancia
    if distance < min_distance:
        print("Persona demasiado cerca, retrocediendo...")
        set_velocity_body(vehicle, -gnd_speed, 0, 0)  # Retroceder (descender)
    elif distance > max_distance:
        print("Persona demasiado lejos, acercándose...")
        set_velocity_body(vehicle, gnd_speed, 0, 0)  # Avanzar (ascender)

    # Controlar el movimiento en X (izquierda/derecha)
    if x_center < frame_center - 100:
        print("Moviéndose a la izquierda")
        set_velocity_body(vehicle, 0, -gnd_speed, 0)  # Mover a la izquierda (norte)
    elif x_center > frame_center + 100:
        print("Moviéndose a la derecha")
        set_velocity_body(vehicle, 0, gnd_speed, 0)  # Mover a la derecha (sur)
    
    # Controlar la altura (subir/bajar) en Y
    if y_center < 200:
        print("Subiendo")
        set_velocity_body(vehicle, 0, 0, -gnd_speed)  # Subir (este)
    elif y_center > 400:
        print("Bajando")
        set_velocity_body(vehicle, 0, 0, gnd_speed)  # Bajar (oeste)

#-- Key event function (sin cambios, solo se muestra como ejemplo)
def key(event):
    if event.char == event.keysym:  # Estándar
        if event.keysym == 'r':  # Se va a ir hacia arriba NO activar XD
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
        elif event.keysym == 'w':
            set_velocity_body(vehicle, 0, 0, -gnd_speed)
        elif event.keysym == 's':
            set_velocity_body(vehicle, 0, 0, gnd_speed)
    else:  # No estándar
        if event.keysym == 'Up':
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(vehicle, -gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(vehicle, 0, gnd_speed, 0)
# Conexión a la cámara
stream_url = "http://192.168.143.5:8080/stream.mjpeg?clientId=HX4PsClPt2efOF1T"

cap = cv2.VideoCapture(stream_url)

ret, frame = cap.read()
model = YOLO("HeadDetect.pt")
tracker = Tracker()

colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(10)]
detection_threshold = 0.5

fps = 0
frame_count = 0
start_time = time.time()

while ret:
    frame_count += 1
    results = model.predict(task="detect", source=frame, imgsz=160, max_det=2, conf=0.60, show_labels=False, show_conf=True, save=True, device="cpu", augment=False, verbose=False)

    for result in results:
        detections = []
        for r in result.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = r
            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)
            class_id = int(class_id)

            if score > detection_threshold:
                detections.append([x1, y1, x2, y2, score])

        if detections != []:
            tracker.update(frame, detections)

            for track in tracker.tracks:
                bbox = track.bbox
                x1, y1, x2, y2 = bbox
                track_id = track.track_id
                PerHeight = y2 - y1
                Known_Height = 1.68  # Altura promedio de una persona
                Length_focal = 4850  # Longitud focal de la cámara
                distancia = distance_to_camera(Known_Height, Length_focal, PerHeight)

                # Calcula el centro del objeto
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)

                # Llama a la función de control del dron
                control_drone(x_center, y_center, distancia)

                # Dibuja la caja y la información sobre el frame
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (colors[track_id % len(colors)]), 3)
                cv2.putText(frame, "ID:" + str(track_id), (int(x1), int(y1) - 20), cv2.FONT_HERSHEY_PLAIN, 1, (colors[track_id % len(colors)]), 2)
                cv2.putText(frame, "Distancia:" + str(format(distancia, ".2f")), (int(x1), int(y1) - 7), cv2.FONT_HERSHEY_PLAIN, 1, (colors[track_id % len(colors)]), 2)

    cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

    ret, frame = cap.read()

    if time.time() - start_time >= 1:
        fps = frame_count
        frame_count = 0
        start_time = time.time()

cap.release()
cv2.destroyAllWindows()

# Cerrar la conexión con el vehículo
vehicle.close()
