import serial
import time
import math
import string

success = 0
ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout= 0.5)
#while not success:
	
	
	#if(ser == 0):
		#success = 0
	#else:
		#success = 1

#print (ser.name())
counter = 0
tagw = 0
tagh = 0
tagx = 0
tagy = 0
tag_found = False
#Expand this
# we prob want a routine that will strip out
# from the text the distance, angle etc. and put
# them in handy vars 
while True:
    
	ser.flushInput() #buffer input

	line = ser.readline() #read input
	word = string.split(line, ";") #field split

	if(len(word) > 3):
		tagw = int(word[0])
		tagh = int(word[1])
		tagx = int(word[2])
		tagy = int(word[3])
		tag_found = True
		
	else:
		tagw = 0
		tagh = 0
		tagx = 0
		tagy = 0
		tag_found = False
		
	print(tagw, tagh, tagx, tagy)

