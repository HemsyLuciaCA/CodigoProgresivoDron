
import cv2
from ultralytics import YOLO
import random
from tracker import Tracker
import time

###IMPLEMENtACION
def distance_to_camera(knownHeight, focalLength, perHeight):
    # Compute and return the distance from the marker to the camera
    return (knownHeight * focalLength) / perHeight


cap = cv2.VideoCapture("http://192.168.78.44:8080/stream.mjpeg?clientId=nYRHhvXgnx3E5bAN")

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
    results = model.predict(task="detect", source=frame, imgsz=160, max_det=2, conf=0.60, show_labels=False, show_conf=True, save=False, device="cpu", augment=False, verbose=False)

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
                
        if(detections!=[]):
            tracker.update(frame, detections)
            
            for track in tracker.tracks:
    
                bbox = track.bbox
                x1, y1, x2, y2 = bbox
                track_id = track.track_id
                PerHeight= y2-y1
                Known_Height= 1.68
                Length_focal=4850
                distancia= distance_to_camera(Known_Height,Length_focal,PerHeight)
                
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (colors[track_id % len(colors)]), 3)
                cv2.putText(frame, "ID:"+str(track_id),(int(x1),int(y1)-20),cv2.FONT_HERSHEY_PLAIN,1, (colors[track_id % len(colors)]), 2)
                cv2.putText(frame, "Distancia:"+str(format(distancia, ".2f")),(int(x1),int(y1)-7),cv2.FONT_HERSHEY_PLAIN,1, (colors[track_id % len(colors)]), 2)


    #cv2.putText(frame, f'FPS: {fps}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #cv2.imshow("frame", frame)


    ###IMPLEMENTACION DE DISTANCIA PERSONA-CAM
    #PerHeight= y2-y1
    #Known_Height= 1.68
    #Length_focal=4850
    #distancia= distance_to_camera(Known_Height,Length_focal,PerHeight)
    #cv2.putText(frame, f'Distancia: {distancia}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("frame", frame)

    ###FIN IMPLEMENTACION


    if cv2.waitKey(1) & 0xff == 27:
        break


    ret, frame = cap.read()

    if time.time() - start_time >= 1:
        fps = frame_count
        frame_count = 0
        start_time = time.time()

cap.release()
cv2.destroyAllWindows()
