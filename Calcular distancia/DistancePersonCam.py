# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2

def find_marker(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5,5), 0) #aplica un filtro de desenfoque gaussiano
    edged = cv2.Canny(gray, 35, 125) #detector de bordes de Canny
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #encuentra los contornos en la imagen
    cnts = imutils.grab_contours(cnts) #devuelve la lista de contornos sin importar la versión de OpenCV
    if len(cnts) == 0: #si no encuentra contornos, si la lista esta vacia no hay nada que procesar
        return None #no hay contornos
    c = max(cnts, key=cv2.contourArea)
    return cv2.minAreaRect(c) #devuelve el rectangulo de area minima que encierra el contorno

def distance_to_camera(knownHeight, focalLength, perHeight):
    return (knownHeight*focalLength)/perHeight  #calcula la distancia de los objetos a la camara

def main():
    KNOWN_HEIGHT = 4.5 #guarda la altura real del objeto de referencia que queremos medir en pulgadas
    focalLength = 4850 #guarda la longitud focal de la cámara y 4850 es un ejemplo de valor en pixeles
    cap = cv2.VideoCapture(0) #sirve para abrir la cámara, 0 es la cámara por defecto
    
    if not cap.isOpened(): #si no se abre la cámara
        print("Error: Could not open camera")
        return
    
    while True:
        ret, frame = cap.read() #ret es un booleano que indica si se capturó bine el frame,  frame es la imagen capturada
        
        if not ret:
            print("Error: Could not read frame")
            break
        marker = find_marker(frame)

        if marker is not None:
            inches = distance_to_camera(KNOWN_HEIGHT, focalLength, marker[1][0])

            box = cv2.boxPoints(marker) #calcula las 4 esquinas (x,y) del rectángulo, devuelve un array con los cuatro puntos en float
            box = box.astype(int) #convierte los puntos a enteros
            cv2.drawContours(frame, [box], -1, (255, 0, 0), 2) #dibuja el contorno (las 4 líneas) del rectángulo sobre el frame en color verde y grosor 2
            cv2.putText(frame, "%.2fmetros" % (inches/39.37), # convierte pulgadas a metros, formatea con 2 decimales y añade ft
                        (frame.shape[1] - 200, frame.shape[0] -20), #posición del texto en la imagen
                        cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3) #tipo de letra, tamaño, color y grosor del texto

        cv2.imshow("Frame", frame) #abre una ventana y muestra la imagen, Frame es el nombre de la ventana y frame es la imagen que quieres mostrar
        if cv2.waitKey(1) & 0xFF == ord('q'): #espera 1 milisegundo para detectar si se presiono una tecla, ord devuelve el valor ASCII de la tecla q
            break #sale del bucle while true

    cap.release() #libera la cámara
    cv2.destroyAllWindows() #cierra todas las ventanas abiertas por OpenCV

if __name__ == "__main__":
    main()


# Mismo código que DistanceCalcGPT.py pero enfocado en altura (Height) y algunos ajustes pequeños