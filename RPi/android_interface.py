from bluetooth import *
from interface import *
import time
 
class android_interface(interface):
  #  def __init__(self):
    
    def connect(self):
                try:
                    self.server_sock=BluetoothSocket( RFCOMM )
                    self.server_sock.bind(("",4))
                    self.server_sock.listen(4)
                    port = self.server_sock.getsockname()[1]

                    uuid = "00000003-0000-1000-8000-00805f9b34fb"

                    advertise_service( self.server_sock, "SampleServer",
                                       service_id = uuid,
                                       service_classes = [ uuid, SERIAL_PORT_CLASS ],
                                       profiles = [ SERIAL_PORT_PROFILE ], 
#                                      protocols = [ OBEX_UUID ] 
                                        )

                    print "Waiting for connection on RFCOMM channel %d" % port
                    self.btsock, client_info = self.server_sock.accept()
                    secure = client_info[0]
                    if secure != '08:60:6E:A4:E4:D4':
                        print "Unauthorized device, disconnecting..."
                        return 0
                        
                    print "Accepted connection from ", client_info
                    print "Connected to Android"
                    return 1
                except Exception, e:
                    print "Error@BTConnect: %s" %str(e)
                    return 0
         
    def disconnect(self):
	try:
        	self.btsock.close()
        	self.server_sock.close()
	except Exception, e:
		print "Error@BTDisconnect: %s" %str(e)
        
    def writetoBT(self,msg):
                try:
                    self.btsock.send(msg)
                    print "Write to Android: %s" %(msg)
                except Exception, e:
                    print "Error@BTWrite: %s" %str(e)

                    connected = 0
                    connected = self.connect()
                    while connected == 0:
                        print "Attempting reconnection..."
                        self.disconnect()
                        time.sleep(1)
                        connected = self.connect()
 
    def readfromBT(self):
                try:
                    msg = self.btsock.recv(1024)
                    print "Read from Android: %s" %(msg)
                    return (msg + "\n\r")
                except Exception, e:
                    print "Error@BTRead: %s" %str(e)
                    
                    connected = 0
                    connected = self.connect()
                    while connected == 0:
                        print "Attempting reconnection..."
                        self.disconnect()
                        time.sleep(1)
                        connected = self.connect()
