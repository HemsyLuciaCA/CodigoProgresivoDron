import cv2
import subprocess
import numpy as np

# Dirección del stream
url = "http://192.168.68.252:6868"  # O usa "rtsp://..." si es un stream RTSP


# Comando FFmpeg para obtener frames del stream
ffmpeg_command = [
    'ffmpeg',
    '-i', url,                    # URL del stream
    '-f', 'image2pipe',            # Output como un flujo de imágenes
    '-pix_fmt', 'bgr24',           # Formato de píxeles compatible con OpenCV
    '-vcodec', 'rawvideo',         # Usar video sin comprimir (raw video)
    '-'
]

# Inicia el proceso FFmpeg
ffmpeg_process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Dimensiones del frame (ajustar según el stream)
width, height = 640, 480  # Cambia estos valores según el tamaño del stream

while True:
    # Lee el siguiente frame desde stdout de FFmpeg
    raw_frame = ffmpeg_process.stdout.read(width * height * 3)  # Tamaño del frame (640x480, 3 canales de color)
        
    # Si no se recibe el frame completo, o el stream se ha cerrado, termina el bucle
    if len(raw_frame) != width * height * 3:
        break

    # Convierte el frame a un array NumPy
    frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((height, width, 3))

    # Muestra el frame usando OpenCV
    cv2.imshow('Frame', frame)

    # Controla el tiempo entre frames (espera por 1 ms)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera los recursos
ffmpeg_process.stdout.close()
ffmpeg_process.stderr.close()
ffmpeg_process.wait()
cv2.destroyAllWindows()
