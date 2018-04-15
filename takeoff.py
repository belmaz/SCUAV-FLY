from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time, sys, math 

import argparse
parser = argparse.ArgumentParser()
#parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

#Connect to vehicle.
#print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

home_position_set = False
vehicle.armed = True
time.sleep(1)

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt 

#New stuff

# Check that vehicle is armable
while not vehicle.is_armable:
	print(" Battery: %s" % vehicle.battery)
	print(" Waiting for vehicle to initialise...")
	time.sleep(1)
    # If required, you can provide additional information about initialisation
    # using `vehicle.gps_0.fix_type` and `vehicle.mode.name`.
    
#print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed 
#vehicle.armed = True

#print " Vehicle is armed: %s" % vehicle.armed 

#Copter should be in guided mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)
    
print "Vehicle is armed: %s" % vehicle.armed


# ~~~~~~~~~~~~~ this all works ~~~~~~~~~~~~~
print "taking off"
thealt = vehicle.location.global_relative_frame.alt 
thetarg = thealt + 0.5


vehicle.simple_takeoff(thetarg)
while True:
	time.sleep(1)
	print " Alt: %s" % vehicle.location.global_relative_frame.alt 
	if (vehicle.location.global_relative_frame.alt >= thetarg * 0.95):
		print "Reached target alt"
		break

vehicle.mode = VehicleMode("LAND")
vehicle.armed = False
print "Finished"
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




