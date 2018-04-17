import serial

ser = serial.Serial('/dev/ttyACM0', 9600)  # Arduiino baudrate

tag = False

#claw fuction:
# boolean function, takes in if was found value, lets claw know when to open and close 
def claw_function(tagFound):
	global tag
	if (tagFound == True && distance <= 5):
		ser.write('1') # close the claw

	if(tagFound == False):
		ser.write('0') #open the claw and keep it open 

	else:
		print("Invalid command!")	# either no connection or something went wrong!


# Notes:
# tagFound means that the drone is the nope variable. 
# See which position the claw starts in once everything is running.
# If it starts open then it is fine, else we need to fix the code for claw.		

def tagfunc():
	global tag
	ser.read(ch)

	if(ch == 'l'):
		res = ch - 107

	if(ch == 'r'):
		res = ch - 113

	if(ch == 'f'):
		res = ch - 101

	if(ch == 'b'):
		res = ch - 97

	if(ch == 'a'):
		res = ch - 96
		tag = True
	return tag

claw_function(tag)