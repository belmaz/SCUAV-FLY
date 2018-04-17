import serial

ser = serial.Serial('/dev/ttyACM0', 9600)  # Arduiino baudrate

tag = False

#claw fuction:
# boolean function, takes in if was found value, lets claw know when to open and close 
def claw_function(tagFound):
	global tag
	if (tagFound == True and distance <= 5):
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
	ser.flushInput()
	ch = ser.readline()
	print ch

	if(ch[0] == 'l'):
		res = ord(ch[0]) - 107
		print res #left
		ser.write(res)
	else:
		if(ch[0] == 'r'):
			res = ord(ch[0]) - 112
			print res #right
			ser.write(res)
		else:
			if(ch[0] == 'f'):
				res = ord(ch[0]) - 99
				print res #fwd
				ser.write(res)
			else:
				if(ch[0] == 'b'):
					res = ord(ch[0]) - 94
					print res #back
					ser.write(res)
				else:
					if(ch[0] == 'a'):
						res = ord(ch[0]) - 97
						tag = True
						print res #found
						ser.write(res)
					else:
						print "decap mode engaged!!!"
	return tag

while True:
	tagfunc()
	
	claw_function(tag)
