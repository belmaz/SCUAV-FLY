from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

import bluetooth, sys, time, math

#~~~~~~~~~~~~~~~~~~~~~~~~~~~BLUETOOTH~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
BTgood = 0
alldone = 0

altitude = vehicle.location.global_relative_frame.alt

def armIt():
	print("-----------------> ARMING <-----------------")
	vehicle.armed = True
	
def disarmIt():
	print("-----------------> DISARMING <-----------------")
	vehicle.armed = False

def takeoff(target_alt):
	print("------------------> TAKEOFF  <-----------------")
	vehicle.simple_takeoff(target_alt)

	while True:
		time.sleep(1)
		if(vehicle.location.global_relative_frame.alt >= target_alt * 0.95):
			print "reached altitude"
			break

	vehicle.mode = VehicleMode("LAND")

def land():
	vehicle.mode = VehicleMode("LAND")

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)




def connectBT():
	# this is the setup routine that must be called first.
	# It will halt your program execution until it connects to your phone, etc.
	# The rest is asynch
	global BTgood
	global client_sock
	global server_sock	
	if (BTgood == 0):
		# init and connect bt
		print("Attempting to connect to your bluetooth device")
		print("  (you need to initiate it from your phone....)")
		server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

		port = 1
		server_sock.bind(("",port))
		server_sock.listen(1)

		client_sock,address = server_sock.accept()
		print "Accepted connection from ",address
		client_sock.setblocking(0)
		
		BTgood = 1
	else:
		client_sock.close()
		server_sock.close()

def closeBT():
	BTgood = 0
	client_sock.close()
	server_sock.close()

def dispatchBT():
	# bluetooth dispatcher
	global BTgood
	global client_sock
	global server_sock
	global alldone
	if (BTgood == 1):
		try:
			data = client_sock.recv(1)
			print "received [%s]" % data
			#a = a + 1
			if data[0] == 'g': #green arm button
				armIt()
			if data[0] == 'b': #blue disarm button
				disarmIt()
			if data[0] == 'r': #red disconnect button
				alldone =1
			if data[0] == 'u': #up takeoff button
				takeoff(1)
			if data[0] == 'd': #down land button
				land()
		except:
			pass
			#print("Yippeee!")
	else:
		print("There is no bt connection.  Make sure you run connectBT() first")

vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

home_position_set = True
vehicle.armed = False
time.sleep(1)

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
#print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt 

#New stuff

# Check that vehicle is armable
while not vehicle.is_armable:
	#vehicle.gps_0.fix_type = 0
	print " GPS: %s" % vehicle.gps_0
	#print(" Battery: %s" % vehicle.battery)
	print(" Waiting for vehicle to initialise...")
	time.sleep(1)
    # If required, you can provide additional information about initialisation
    # using `vehicle.gps_0.fix_type` and `vehicle.mode.name`.
    

#print " Vehicle is armed: %s" % vehicle.armed 

#Copter should be in guided mode
vehicle.mode = VehicleMode("GUIDED")
#print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed 
#vehicle.armed = True
connectBT()


#while not vehicle.armed:
#    print " Waiting for arming..."
#    time.sleep(1)
    
#print "Vehicle is armed: %s" % vehicle.armed

while not alldone:
	#time.sleep(1)
	#print " Alt: %s" % vehicle.location.global_relative_frame.alt 
	#print " GPS: %s" % vehicle.gps_0
	#print(" Global Location: %s" % vehicle.location.global_frame)
	#print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
	#print(" Local Location: %s" % vehicle.location.local_frame)
	dispatchBT()

closeBT()
