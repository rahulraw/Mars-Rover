import serial
import struct
import rospy
from  time import sleep
from joystick_packages.msg import Controller

class Arduino:
    def __init__(self):
        self.ACCEPT_SERIAL = 255
        self.SEND_SERIAL = 254

        rospy.init_node('serial_bridge', anonymous=True)
        self.setup_topics()
        self.ser = serial.Serial('/dev/ttyACM2', 9600)
        self.rate = rospy.Rate(10)
        self.controller = Controller()

    def callback(self, controller):
        if controller.left_joy_x != self.controller.left_joy_x or controller.left_trigger != self.controller.left_trigger:
            self.controller = controller

    def start(self):
        while not rospy.is_shutdown():
            msg = self._read_byte()
            if (msg == self.ACCEPT_SERIAL):
                self.claw();
            elif (msg == self.SEND_SERIAL):
                topic = self._read_byte()
                length = self._read_byte()
                data = [self._read_byte() for i in range(length)]
                # Data needs to be passed into our other message

    def claw(self):
        if self.controller.left_trigger == 0:
            self.ser.write(chr(1))
            self.ser.write(chr((self.controller.left_joy_x + 90) / 2))
        
    def setup_topics(self):
        rospy.Subscriber("RCValues", Controller, self.callback, queue_size=1)

    def _read_byte(self):
        return struct.unpack('B', self.ser.read(1))[0]

if __name__ == '__main__':
    arduino = Arduino()
    arduino.start()
