from dronekit import connect
import time

connection_string = 'udp:172.17.168.133:19980'
vehicle = connect(connection_string, wait_ready=True)
print("Intentando conectar ...")

# Asegúrate de que el gimbal esté disponible
if vehicle.gimbal is None:
    print("El gimbal no está disponible.")
else:
    print("Gimbal disponible y listo.")

# Función para controlar el gimbal
def controlar_gimbal(pitch, yaw, roll):
    """
    Controla el gimbal del drone.
    
    Args:
    pitch: Ángulo de pitch en grados (positivo hacia abajo, negativo hacia arriba)
    yaw: Ángulo de yaw en grados (positivo hacia la derecha, negativo hacia la izquierda)
    roll: Ángulo de roll en grados (positivo hacia la derecha, negativo hacia la izquierda)
    """
    print(f"Controlando gimbal: Pitch={pitch}, Yaw={yaw}, Roll={roll}")
    
    # Rotar el gimbal
    vehicle.gimbal.rotate(pitch, yaw, roll)

# Controlar el gimbal
controlar_gimbal(pitch=-45, yaw=0, roll=0)  # Ejemplo: gimbal mirando hacia abajo

# Mantener el control durante 5 segundos
time.sleep(5)

# Restaurar posición original del gimbal
controlar_gimbal(pitch=0, yaw=0, roll=0)  # Gimbal en posición neutra

# Desconectar
vehicle.close()
