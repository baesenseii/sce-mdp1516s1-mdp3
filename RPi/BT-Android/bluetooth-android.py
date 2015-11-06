#!/usr/bin/python

import os,commands,time,serial,sys

bd_addr = ""
channel = "5"
port = "rfcomm5"
bluetoothSerial = None

def initialise():
	global bluetoothSerial

	cmd1 = "nohup rfcomm connect "+port+" "+bd_addr+" "+channel+" > /dev/null 2>&1 &"
	cmd2 = "sdptool browse "+bd_addr+" | grep 'Test Service'"

	while (1):
		a = commands.getoutput(cmd2)
		
		if len(a) == 0:
			a = commands.getoutput(cmd2)

		else:
			break

	os.system(cmd1)
	time.sleep(1)

	if os.path.exists('/dev/rfcomm5'):
		bluetoothSerial = serial.Serial('/dev/'+port, baudrate=9600)

	else:
		print "Bluetooth device not ready! System exiting!"
		Sys.exit(0)


def sendData():
	global bluetoothSerial

	a = None

	while a == None:
		try:
			a = raw_input("Please enter value to send (type exit to stop): ")

		except:
			pass

		if a == "exit":
			print "Ending system!"

			sys.exit(0)


	try:
		bluetoothSerial.write("{0}".format(a))

	except serial.SerialException:
		print "Bluetooth connection error! Exiting!"

def recvData(size):

	global bluetoothSerial

	try:
		rcv = bluetoothSerial.read(size)
		print "Data received: "+rcv
	
	except serial.SerialException:
		print "Error receiving data! System will shutdown!"


initialise()

while (True):
	if bluetoothSerial.inWaiting() > 0:
		recvData(bluetoothSerial.inWaiting())
	else:
		sendData()