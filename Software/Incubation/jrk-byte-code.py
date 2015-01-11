## This code if for the JRK21v3 motor controller used for the arm ##
import sys

def main(self):
    device = '/dev/ttyACM0'
    fd = open(device, O_RDWR or O_NOCTTY)
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

def jrkGetVariable(self,fd,command):
    if(write(fd,&command,1) ==-1):
        OSError("error writing")
        return -1

    response = []
    if (read(fd,response,2) not ==2):
        OSError("error reading")
        return -1

    return response[0] + 256*response{1] 

def jrkGetFeedback(self,fd):
    return jrkGetVariable(fd,'0xa5')

def jrkGetTarget(self,fd):
    return jrkGetVariable(fd,'0xa3')

def jrkSetTarget(self,target):
    command = [0xc0 + (target & 0x1f), (target >> 5) & 0x7f]
    if (write(fd, command, sys.getsizeof(command)) ==-1):
        OSError("error writing")
        return -1

    return 0





