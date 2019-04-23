import socket
import sys
from ctypes import *
import getch

v = 0
t = 0

UDP_IP = "192.168.1.99"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.connect((UDP_IP, UDP_PORT))

class Robot(Structure):
	_fields_ = [("velocity", c_double),("theta", c_double),("mode", c_int)] #setting up c type struct
	
class Data(Structure):
	_fields_ = [("X", c_double),("Y", c_double),("phi", c_double)]

print("Welcome, the controls for the robot are:")
print("q is to exit")
print("r is to restart the robot")
print("n is receive data back from the robot")
print("Space bar is to stop")
print("a is to servo 0 deg")
print("s is to servo 90 deg")
print("d is to servo 180 deg")
print("f is to servo 270 deg")
print("Up arrow is to increase speed")
print("Down arrow is to decrease speed")
while True:
	b = getch.getch() #get theta term
	print("b is equal to: ") #for testing
	print(b) #for testing
	if (b == 'a'):
		t = 0
		print("t is equal to: ")
		print(t)
	elif (b == 's'):
		t = 90
		print("t is equal to: ")
		print(t)
	elif (b == 'd'):
		t = 180
		print("t is equal to: ")
		print(t)
	elif (b == 'f'):
		t = 270
		print("t is equal to: ")
		print(t)
	if (b == 'r'): #restart check
		sendRobot = Robot(0,0,1) #parse data
		sock.send(sendRobot) #send parse data
	if (b == 'n'): #restart check
		sendRobot = Robot(v,t,2) #parse data
		sock.send(sendRobot) #send parse data
		buff = sock.recv(sizeof(Data))
		myData = Data.from_buffer_copy(buff)
		print "X=%d, Y=%d, phi=%f" % (myData.X, myData.Y, myData.phi)
	if (b == ' '):
		sendRobot = Robot(0,0,0) #parse data
		sock.send(sendRobot) #send parse data
	if (b == 'q'):
		print("Exiting")
		sys.exit()
	else:
		if (getch.getch() == '\033'): #get velocity term
		getch.getch()
		c = getch.getch()
		if (c == 'A'):
			v += 17
			if (v > 255):
				v = 255
			print("v is equal to: ") #for testing
			print(v) #for testing
		elif (c == 'B'):
			v -= 17 
			if (v < 0):
				v = 0
			print("v is equal to: ") #for testing
			print(v) #for testing
		sendRobot = Robot(v,t,0) #parse data
		sock.send(sendRobot) #send parse data