import serial
import struct 
import time

class Jrk:
    NO_BYTES_WRITTEN = -1
    NO_BYTES_READ = -2
    TIMEOUT_READING = -3
    TIMEOUT_WRITING = -4

    def __init__(self, device):
        self.port = serial.Serial(device, baudrate=9600, timeout=1)
        self.direction = 0
        self.stopped = True
        self.motorStop()

    def set_direction(self, direction):
        self.direction = direction

    def run(self):
        if not self.direction and not self.stopped:
            self.stopped = True
            self.motorStop()
        elif self.direction:
            self.stopped = False
            target = int(2000 + 2000 * self.direction)
            self.setTarget(target)

    def writeByte(self, command):
        try:
            numBytes = self.port.write(struct.pack('<B', command))
        except SerialTimeoutException:
            return TIMEOUT_WRITING

        if numBytes == 0:
            return self.NO_BYTES_WRITTEN

        return 0

    def getVariable(self, command):
        result = self.writeByte(command)
        if (result != 0):
            return result

        bytesRead = self.port.read(2)

        if bytesRead == '':
            return self.NO_BYTES_READ

        return struct.unpack('<H', bytesRead)[0]

    def setTarget(self, target):
        byte1 = 0xC0 + (target & 0x1F)
        byte2 = (target >> 5) & 0x7F
        result = self.writeByte(byte1);
        if result != 0:
            return result

        result = self.writeByte(byte2);
        if result != 0:
            return result

        return 0
    
    def motorStop(self):
        byte1 = 0xFF 
        result = self.writeByte(byte1);
        if result != 0:
            return result

        return 0

    def getFeedBack(self):
        self.port.flushInput()
        return self.getVariable(0xA5)

    def getTarget(self):
        return self.getVariable(0xA3)

    #Get the errors and clear them
    def getErrorFlagsHalting(self):
        return self.getVariable(0xB3)
