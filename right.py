from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time, sys, math 

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~TEMPLATE~~~~~~~~~~~~~~~~~~~~~~~~~
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

while not vehicle.is_armable:
	print("battery: %s" % vehicle.battery)
	print("waiting to initialize")
	time.sleep(1)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.not_armed:
	print("Waiting to arm")
	time.sleep(1)

print("vehicle is now armed: %s" % vehicle.armed)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~NEW STUFF(TURN LEFT)~~~~~~~~~~~~~~~
def send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#~~~~~~~~~~TAKEOFF~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
target = 1
vehicle.simple_takeoff(target)

while True:
	print("Altitude: %s", vehicle.location.global_relative_frame.alt)
	if(vehicle.location.global_relative_frame.alt >= target):
		print("Reached target alt")
		break

	time.sleep(1)

velx = -0.5
vely = 0
velz = 0
duration = 5