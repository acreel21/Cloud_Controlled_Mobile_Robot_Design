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

while True:
	a = getch.getch() #get velocity term
	c = getch.getch() #get velocity term
	if (a == 'r'): #restart check
		sendRobot = Robot(0,0,1) #parse data
		sock.send(sendRobot) #send parse data
		print("a is equal to: ") #for testing
		print(a) #for testing
	else:
		print("c is equal to: ") #for testing
		print(c) #for testing
		if (c == 'H'):
			v = 127 
			print("v is equal to: ") #for testing
			print(v) #for testing
		elif (c == 'P'):
			v = 0 
			print("v is equal to: ") #for testing
			print(v) #for testing
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
		sendRobot = Robot(v,t,0) #parse data
		sock.send(sendRobot) #send parse data