from pymavlink import mavutil

# Set up connection to vehicle
master = mavutil.mavlink_connection('udp:localhost:5600')
print("conectado")

# Wait for the heartbeat message to ensure connection
master.wait_heartbeat()
print("heartbeat recibido")

# Get list of available MAVLink messages
msg_names = master.messages.keys()

# Print out list of available messages
print("Available MAVLink messages:")
for name in msg_names:
    print("- {}".format(name))