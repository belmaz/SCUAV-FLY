from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

import bluetooth, time, sys, math 

BTgood = 0
alldone = 0


def armIt():
	print("-----------------> ARMING <-----------------")
	vehicle.armed = True
	
def disarmIt():
	print("-----------------> DISARMING <-----------------")
	vehicle.armed = False


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
			if data[0] == 'g':
				armIt()
			if data[0] == 'b':
				disarmIt()
			if data[0] == 'r': #red disconnect button
				alldone =1
		except:
			pass
			#print("Yippeee!")
	else:
		print("There is no bt connection.  Make sure you run connectBT() first")


#Connect to vehicle.
#print 'Connecting to vehicle on: %s' % args.connect
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
"""
# ~~~~~~~~~~~~~ this all works ~~~~~~~~~~~~~
print "taking off"
thealt = vehicle.location.global_relative_frame.alt 
thetarg = thealt + 0.5


vehicle.simple_takeoff(thetarg)
while True:
	time.sleep(1)
	print " Alt: %s" % vehicle.location.global_relative_frame.alt 
	print " GPS: %s" % vehicle.gps_0
	if (vehicle.location.global_relative_frame.alt >= thetarg * 0.95):
		print "Reached target alt"
		break

vehicle.mode = VehicleMode("LAND")
"""
vehicle.close()
	
"""

le True:
	##print " Alt: %s" % vehicle.location.global_relative_frame.alt 
	#print " System status: %s" % vehicle.system_status.state
	##print " GPS: %s" % vehicle.gps_0
	#vehicle.wait_ready('autopilot_version



#def PX4setMode(mavMode):
    #vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               #mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               #mavMode,
                                               #0, 0, 0, 0, 0, 0)"""




