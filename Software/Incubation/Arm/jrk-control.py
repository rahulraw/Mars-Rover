import serial
import struct 
import time

NO_BYTES_WRITTEN = -1
NO_BYTES_READ = -2
TIMEOUT_READING = -3
TIMEOUT_WRITING = -4

def writeByte(port, command):
    try:
        numBytes = port.write(struct.pack('<B', command))
    except SerialTimeoutException:
        return TIMEOUT_WRITING

    if numBytes == 0:
        return NO_BYTES_WRITTEN

    return 0

#used to write get the two bytes variables from polulu
def jrkGetVariable(port, command):

    result = writeByte(port, command)
    if (result != 0):
        return result

    bytesRead = port.read(2)

    if bytesRead == '':
        return NO_BYTES_READ

    return struct.unpack('<H', bytesRead)[0]

def jrkSetTarget(port, target):
    
    byte1 = 0xC0 + (target & 0x1F)
    byte2 = (target >> 5) & 0x7F

    result = writeByte(port, byte1);
    if result != 0:
        return result

    result = writeByte(port, byte2);
    if result != 0:
        return result

    return 0

def jrkGetFeedBack(port):
    port.flushInput()
    return jrkGetVariable(port, 0xA5)

def jrkGetTarget(port):
    return jrkGetVariable(port, 0xA3)

#Get the errors and clear them
def jrkGetErrorFlagsHalting(port):
    return jrkGetVariable(port, 0xB3)

def main():
    port = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=1)

    jrkGetErrorFlagsHalting(port)

    target = 200

    print(jrkGetTarget(port))
    print(jrkSetTarget(port, target))

    feedback = jrkGetFeedBack(port)
    while(feedback != target):
        jrkSetTarget(port, target)
        feedback = jrkGetFeedBack(port)
        print(feedback)
        time.sleep(0.25)

main()