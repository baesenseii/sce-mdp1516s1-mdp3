import sys
import thread
import time
import threading
import serial
import os
 
from interface import *
 
class arduino_interface(interface):
	def __init__(self):
		self.ser = serial.Serial()
        self.baudrate = 115200
            #self.baudrate = 9600
		self.ser = None
	
	def connect(self):
		try:
			print "Trying to connect Arduino..."
			self.ser = serial.Serial('/dev/ttyACM0',self.baudrate, timeout = 2)
			if (self.ser != None):
				print "Connected to Arduino."
				return 1
		except Exception, e:
			print "Error@Arduino Connection: %s" %str(e)
			return 0
 
	def disconnect(self):
		self.ser.close()

	def flush(self):
		self.ser.flushInput()
		self.ser.flushOutput()
 
	def readfromSR(self):
		try:
			msg = self.ser.readline()
			if msg != "":
				print "Read from Arduino: %s" %(msg)
				return msg
		except Exception, e:
				print "Error@Arduino Read: %s" %str(e)
	
		#ser.flushInput()
 
	def writetoSR(self,msg):
		try:
			self.ser.write(msg)
			print "Write to Arduino: %s" %(msg)
		except Exception, e:
				print "Error@Arduino Write: %s" %str(e)
		 
