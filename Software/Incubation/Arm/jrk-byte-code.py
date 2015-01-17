## This code if or the JRK21v3 motor controller used for the arm ##
import sys
import serial
import struct
import time
import math
import os

def main():
    device = serial.Serial("/dev/ttyACM0", baudrate = 38400, timeout = 1)
    rcv = device.read(32)
    print (repr(rcv))
    fd = open(device, os.O_RDWR | os.O_NOCTTY)
    if (fd == -1):
        OSError(device)
        return 1

    feedback = jrkGetFeedback(fd)
    print("Current Feedback is %d./n" % feedback)

    target = jrkGetTarget(fd)  
    print("Current Target is %d./n" % target)

    newTarget = 3000 if target < 2048 else 1000
    print("Setting Target to %d./n" % newTarget)
    jrkSetTarget(fd, newTarget)
    close(fd)
    return 0

def jrkGetVariable(fd,command):
    if(write(fd,command,1) ==-1):
        OSError("error writing")
        return -1

    response = []
    if (read(fd,response,2) !=2):
        OSError("error reading")
        return -1

    return response[0] + 256*response[1] 

def jrkGetFeedback(fd):
    return jrkGetVariable(fd,'0xa5')

def jrkGetTarget(fd):
    return jrkGetVariable(fd,'0xa3')

def jrkSetTarget(target):
    command = [0xc0 + (target & 0x1f), (target >> 5) & 0x7f]
    if (write(fd, command, sys.getsizeof(command)) ==-1):
        OSError("error writing")
        return -1

    return 0

x = main()
print x


