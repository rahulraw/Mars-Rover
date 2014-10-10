# TCP Client Code	
from socket import *   # imports socket module

host = "localhost" #set the server address to variable host
port = 4446 #sets the variable port to 4446

s = socket(AF_INET, SOCK_STREAM)   # creates a socket
s.connect((host,port))   # connect to server address
msg=s.recv(1024)  #  receives data upto 1024 bytes and stores in variable msg

print "Message from server : " + msg    

s.close()   #closes socket
