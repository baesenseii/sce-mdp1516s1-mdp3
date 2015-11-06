#!/usr/bin/python

#imports the Python Serial library (needs to be installed)

import serial


#creates a Serial object that references to the USB Serial port

ser = serial.Serial('/dev/ttyACM0',9600)


#Writes the value '1', which will be sent to the Arduino.

ser.write('1')