import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGLobal #para comunicacion con el dron
from pymavlink import mavutil #para mensajes crudos MAVLink, estructura del protocolo, más que funciones que DroneKit ya tiene

print("connecting....")

#vehicle=connect('udp:192.168.137.161:199w0')
vehicle=connect('udp:192.168.155.28:19980', wait_ready=True, heartbeat_timeout=60) #conexión con el dron (ip y puerto de la laptop)

gnd_speed = 0.25 #velocidad base estándar en m/s para movimientos
print("connecting..2..")
#--Arm and takeoff
def arm_and_takeoff(altitude):

    #while not vehicle.is_armable:
    #    print("Aun no puede arrancar")
    #    time.sleep(1)

    print("Arrancando motores")
    vehicle.mode=VehicleMode("GUIDED")
    vehicle.armed=True

    while not vehicle.armed:time.sleep(1)

    print("Despegando")
    vehicle.simple_takeoff(altitude) #despega y sube a altitud objetivo

    while True: #bucle que verifica la altitud de 5 metros
        v_alt = vehicle.location.global_relative_frame.alt
        print(">>Altitude = %.lf m" % v_alt)
        break
        if v_alt>=altitude-1.0:
            print("Objetivo de altitud alcanzado")
            break
        time.sleep(5)

def set_velocity_body(vehicle, vx, vy, vz): #vz, negativo = subir, positivo = bajar
    """ Remember: vz es la velocidad de descenso!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html #-- Documentación de referencia, se explica como mandar comandos de posición y velocidad al dron en modo GUIDED
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, # posiciones
    bit 4: vx, bit 5: vy, bit 6: vz, # velocidades
    bit 7: ax, bit 8: ay, bit 9: az, # aceleraciones  
    """ #cadena de bits que indica qué dimensiones deben ser ignoradas por el vehículo
        #0b0000111111000111 -> Considera solo las velocidades
        #0b0000000000000111 -> Considera solo las posiciones
        #0b0000111111111000 -> Considera solo las aceleraciones
        #0b0000000000000000 -> Considera todo
        # Si esta en 1, ignora la dimensión
        # Si esta en 0, toma en cuenta la dimensión

    msg = vehicle.message_factory.set_position_target_local_ned_encode( #se crea el mensaje MAVLink crudo, Set_position_target_local_ned establece la posición y la velocidad en NED
            #NED (North, East, Down) es un sistema de coordenadas relativo al punto de inicio del vehículo usado en drones
            0, #time_boot_ms (tiempo que arrancó el autopiloto en milisegundos, lo ignoramos)
            0, 0, #target system, target component (ID del sistema y componente al que se envía el mensaje, 0,0 es el vehículo completo)
            mavutil.mavlink.MAV_FRAME_BODY_NED, #marco de referencia, BODY_NED indica los ejes del cuerpo del dron
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY, velocidades en el marco BODY_NED
            0, 0, 0,        #-- ACCELERATIONS
            0, 0) #rumbo y velocidad de giro (yaw, yaw_rate), los ignoramos
    vehicle.send_mavlink(msg) #envia el mensaje al autopiloto por UDP
    vehicle.flush() #se asegura que el mensaje se envíe inmediatamente
    
#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(5) #despega y sube a 5 metros de altitud


#Controlando con la mano
while True:
    with open('command.txt', 'r') as f:
        command=f.read().strip() #lee el comando del archivo de texto que cambia según lo que escribe mediapipe 
    print("a") #imprime una a cada ciclo para saber que el bucle está corriendo
    time.sleep(1) #lee el archivo cada segundo
    if command == 'Up':
        print("Arriba") #en realidad va a ir hacia adelante
        set_velocity_body(vehicle, 0, 0, -gnd_speed)
    #elif command == 'r': SE VA A IR HACIA ARRIBA NOOO ACTIVAR
    #        print("\n\n\bModo RTL seleccionado--> sigua con la misma señal para confirmar")
    #        time.sleep(1)
    #        with open('/home/liafiis/Escritorio/command.txt', 'r') as f:
    #            commandcon=f.read().strip()
    #        if commandcon == 'r':
    #            print(">>>>>> Set the vehicle to RTL")
    #            vehicle.mode = VehicleMode("RTL")  
    #            time.sleep(25) 
    elif command == 'Down':
        print("abajo") #en realidad va a ir hacia atrás
        set_velocity_body(vehicle,0, 0, gnd_speed)
    elif command == 'Left':
        print("izquierda")
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
    elif command == 'Right':
        print("derecha")
        set_velocity_body(vehicle, 0, gnd_speed, 0)