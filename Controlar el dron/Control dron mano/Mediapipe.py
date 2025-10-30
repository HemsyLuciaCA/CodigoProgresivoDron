import cv2 #importa OpenCV, procesa el video de la cámara y las imágenes
import mediapipe as mp #importa MediaPipe, biblioteca de aprendizaje automático para procesamiento de imágenes y video
mp_drawing = mp.solutions.drawing_utils #utilidades de dibujo de MediaPipe para superponer anotaciones en imágenes
mp_drawing_styles = mp.solutions.drawing_styles #estilos de dibujo predefinidos para anotaciones de MediaPipe
mp_hands = mp.solutions.hands #solución de MediaPipe para la detección y el seguimiento de manos basado en Machine Learning
import time #para medir tiempos aunque no se observa su uso en el código

# For webcam input:
cap = cv2.VideoCapture(0) #crea un objeto para abrir la cámara por defecto(0) y capturar video

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #ancho deseado del fotograma a 1280 pixeles

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024) #alto deseado del fotograma a 1024 pixeles
with mp_hands.Hands( #inicializa la solución de manos de MediaPipe con parámetros detallados a continuación:
    max_num_hands=1, #limita la detección a una sola mano
    model_complexity=0, #Usa el modelo más rápido y más ligero
    min_detection_confidence=0.5, #umbral de confianza al 50% para considerar una detección válida
    min_tracking_confidence=0.5) as hands: #umbral de confianza al 50% para continuar el seguimiento entre frames
  while cap.isOpened(): #bucle que continúa mientras la cámara esté abierta
    success, image = cap.read() #captura un frame de la cámara, tupla con dos valores: success (bool/verdadero=éxito) y la imagen capturada como un NumPy si tuvo éxito
    if not success: #si success falso, no se pudo capturar el frame
      print("Ignoring empty camera frame.") #ignorando frame vacío de la cámara
      # If loading a video, use 'break' instead of 'continue'.
      continue #continúa el ciclo del bucle sin procesar más este frame

    image.flags.writeable = False #la imagen no puede ser modificada, optimiza el rendimiento al pasarla a MediaPipe
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) #convierte la imagen de BGR (formato OpenCV) a RGB (formato MediaPipe), la imagen se guarda en la variable image
    results = hands.process(image) #procesa la imagen con el modelo de detección de manos de MediaPipe para detectar y rastrear manos en la imagen

    image.flags.writeable = True #la imagen puede ser modificada nuevamente
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # convierte la imagen de RGB de nuevo a BGR para su visualización con OpenCV
    if results.multi_hand_landmarks: #si se detectan manos en la imagen sigue con el flujo
        for hand_landmarks in results.multi_hand_landmarks: #itera sobre cada mano detectada en el frame, solo procesa una mano debido a max_num_hands=1
            mp_drawing.draw_landmarks( #dibuja las marcas de la mano y las conexiones en la imagen
                image, #imagen donde se dibujan los landmarks
                hand_landmarks, #21 puntos de referencia de la mano detectada
                mp_hands.HAND_CONNECTIONS, #Define las conexiones entre los puntos de referencia de la mano
                mp_drawing_styles.get_default_hand_landmarks_style(), #estilo para los puntos (color, tamaño y forma de los puntos)
                mp_drawing_styles.get_default_hand_connections_style()) #estilos para las líneas de conexión entre los puntos (color y grosor)
            
            
            Wrist = [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y] #cada punto se guarda como lista [x,y], coordenadas normalizadas entre 0 y 1, #muñeca
            Thumb_cmc = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].y] #x, posición horizontal del punto (0=izquierda, 1=derecha)
            Thumb_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y] #y, posición vertical del punto (0=arriba, 1=abajo)
            Thumb_ip = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y] #Thumb: puntos del pulgar
            Thumb_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y]
            Index_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y] #Index: puntos del dedo índice
            Index_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y]
            Index_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y]
            Index_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y]
            Middle_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y] #Middle: puntos del dedo medio
            Middle_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y]
            Middle_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y]
            Middle_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y]
            ring_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y] #Ring: puntos del dedo anular
            ring_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].y]
            ring_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].y]
            ring_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y]
            pinky_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y] #Pinky: puntos del dedo meñique
            pinky_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].y]
            pinky_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP].y]
            pinky_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y]

            mcp_delta_y = abs(pinky_mcp[1] - Index_mcp[1]) #mide la inclinación lateral de la mano, calcula la diferencia vertical entre los puntos MCP del meñique e índice
            wrist_to_mcp_y = abs(Middle_mcp[1] - Wrist[1]) #mide altura de la palma, calcula la diferencia vertical entre la muñeca y el punto MCP del dedo medio
            wrist_to_mcp_x = abs(Middle_mcp[0] - Wrist[0]) #mide el ancho de la palma, calcula la diferencia horizontal entre la muñeca y el punto MCP del dedo medio
            hands_orient_param = mcp_delta_y/wrist_to_mcp_y #relación entre inclinación lateral y altura de la palma
            orient_ratio = wrist_to_mcp_x/wrist_to_mcp_y #relación entre ancho y altura de la palma

            # Cálculo de deltas para determinar la posición/movimiento de los dedos
            hands_x_side = Wrist[0] - Middle_mcp[0]          # Dirección horizontal de la mano
            tip_mcp_middle_delta_y = Middle_tip[1]-Middle_mcp[1]  # Flexión vertical dedo medio
            tip_mcp_middle_delta_x = Middle_tip[0]-Middle_mcp[0]  # Movimiento horizontal dedo medio

            tip_mcp_index_delta_y = Index_tip[1] - Index_mcp[1]   # Flexión vertical índice
            tip_mcp_index_delta_x = Index_tip[0] - Index_mcp[0]   # Movimiento horizontal índice

            tip_pip_middle_delta_x = Middle_tip[0]-Middle_pip[0]   # Flexión horizontal dedo medio desde PIP
            tip_pip_index_delta_x = Index_tip[0]-Index_pip[0]      # Flexión horizontal índice desde PIP
        
        ## change the directory according to your sdk build file
        with open('command.txt', 'w') as f: # write command to file


            if orient_ratio <0.5: #orientación de la mano, relación entre el ancho y alto de la palma
                print('normal') #el ancho es menos de la mitad de la altura, mano vertical
                if tip_mcp_middle_delta_y >0: #verifica si el dedo medio está levantado o bajado (bajado)
                    if tip_mcp_index_delta_y<0: #verifica si el dedo índice está levantado o bajado, si está levantado entonces mueve hacia arriba
                        print('cUp')
                        f.write("Up") #dedo medio abajo + dedo índice arriba = mueve hacia arriba
                    else:
                        print('cDown')
                        f.write("Down") #dedo medio abajo + dedo índice abajo = mueve hacia abajo
                else:
                    print('cBack') #dedo medio arriba = retrocede
                    f.write("r")
            else:
                print('parallel')
                if hands_x_side <0:
                    #left
                    if tip_pip_middle_delta_x >0:
                        print('cBack')
                        f.write("r")
                    else:
                        if tip_pip_index_delta_x>0:
                            print('cLeft')
                            f.write("Left")
                        else:
                            print('cDown')
                            f.write("Down")
                
                else:
                    #rigth
                    if tip_pip_middle_delta_x <0:
                        print('cBack')
                        f.write("r")
                    else:
                        if tip_pip_index_delta_x<0:
                            print('cRight')
                            f.write("Right")
                        else:
                            print('cDown')
                            f.write("Down")
            
    else:
        print('bae') #no se dectecta mano
        with open('/home/labia-004/Descargas/Dron Códigos/command.txt', 'w') as f:
            f.write("n")
    
    # image = cv2.resize((image, (1200, 1000)))
    f.close()
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27: #espera 5 ms por una tecla, si se presiona 'Esc' (código ASCII 27), sale del bucle
      break

cap.release() #libera el objeto de captura de video y cierra la cámara
cv2.destroyAllWindows() #cierra todas las ventanas de OpenCV abiertas
