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
vehicle.armed = False
time.sleep(1)

points = [None] * 3

def fly(targetalt):
	#arm vehicle and fly to target altitide
	
	print("pre-arm checks")
	
	#do not arm until autopilot is ready
	while not vehicle.is_armable:
		print("waiting for vehicle to initialise..")
		time.sleep(1)
	
	print("Arming vehicle")
