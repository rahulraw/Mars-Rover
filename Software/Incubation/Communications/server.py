from socket import *

host = "0.0.0.0"
port = 4446

s = socket(AF_INET, SOCK_STREAM)

s.bind((host, port))

s.listen(1)

print "Listening for connections..."
q, addr = s.accept()
print q

data = raw_input("Enter data to be sent: ")

q.send(data)
s.close()
