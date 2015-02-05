import serial
import struct 
import time

class Jrk:

    NO_BYTES_WRITTEN = -1
    NO_BYTES_READ = -2
    TIMEOUT_READING = -3
    TIMEOUT_WRITING = -4

    def __init__(self, device):
        self.port = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=1)

    def writeByte(self, command):
        try:
            numBytes = self.port.write(struct.pack('<B', command))
        except SerialTimeoutException:
            return TIMEOUT_WRITING

        if numBytes == 0:
            return self.NO_BYTES_WRITTEN

        return 0

    #used to write get the two bytes variables from polulu
    def jrkGetVariable(self, command):
        result = self.writeByte(command)
        if (result != 0):
            return result

        bytesRead = self.port.read(2)

        if bytesRead == '':
            return self.NO_BYTES_READ

        return struct.unpack('<H', bytesRead)[0]

    def jrkSetTarget(self, target):
        byte1 = 0xC0 + (target & 0x1F)
        byte2 = (target >> 5) & 0x7F

        result = self.writeByte(byte1);
        if result != 0:
            return result

        result = self.writeByte(byte2);
        if result != 0:
            return result

        return 0

    def jrkGetFeedBack(self):
        self.port.flushInput()
        return self.jrkGetVariable(0xA5)

    def jrkGetTarget(self):
        return self.jrkGetVariable(0xA3)

    #Get the errors and clear them
    def jrkGetErrorFlagsHalting(self):
        return self.jrkGetVariable(0xB3)
