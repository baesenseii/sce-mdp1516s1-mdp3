#!/usr/bin/python

#imports the Python Serial library (needs to be installed)

import serial


#creates a Serial object that references to the USB Serial port

ser = serial.Serial('/dev/ttyACM0',9600)


#the line below will just output whatever information that is sent to the RPi from Arduino

while 1:
	print ser.readline()