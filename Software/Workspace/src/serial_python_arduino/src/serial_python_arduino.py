#!/usr/bin/python
import serial
import struct
import rospy
from  time import sleep
from std_msgs.msg import Bool
from joystick_packages.msg import Controller
from steering.msg import HomingInfo

class Arduino:
    def __init__(self):
        self.ACCEPT_SERIAL = 255
        self.SEND_SERIAL = 254

        rospy.init_node('serial_bridge', anonymous=True)
        self.setup_topics()
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.rate = rospy.Rate(10)
        self.controller = Controller()
        self.shut_down = False
        self.send_shut_down = False
        self.message_length = 0
        self.messages = []
        self.publish_nodes = [self.homing]

    def callback_claw(self, controller):
        if controller.killswitch != self.controller.killswitch:
            shut_down = Bool()
            shut_down.data = self.controller.killswitch == 1
            self.callback_auto_shut_down(shut_down)
        self.controller = controller

    def callback_auto_shut_down(self, shut_down):
        self.send_shut_down = self.shut_down != shut_down.data
        self.shut_down = shut_down.data

    def start(self):
        while not rospy.is_shutdown():
            msg = self._read_byte()
            if (msg == self.ACCEPT_SERIAL):
                self.claw()
                self.auto_shut_down()
                self.send()
            elif (msg == self.SEND_SERIAL):
                topic = self._read_byte()
                length = self._read_byte()
                data = [self._read_byte() for i in range(length)]
                self.publish_nodes[topic - 1](data)
            else:
                print(msg)
                # Data needs to be passed into our other message

    def claw(self):
        if self.controller.left_trigger == 0:
            self.message_length += 1
            self.messages.append(chr(1))
            self.messages.append(chr((self.controller.left_joy_x + 90) / 2))

    def auto_shut_down(self):
        if self.send_shut_down:
            self.message_length += 1
            self.messages.append(chr(2))
            self.messages.append(chr(1 if self.shut_down else 0))
            self.send_shut_down = False

    def homing(self, data):
        homing_info = HomingInfo()
        homing_info.front_left = data[0]
        homing_info.front_right = data[1]
        homing_info.back_left = data[2]
        homing_info.back_right = data[3]

        self.homing_pub.publish(homing_info)

    def send(self):
        self.ser.write(chr(self.message_length))
        [self.ser.write(byte) for byte in self.messages]
        self.message_length = 0
        self.messages = []

    def setup_topics(self):
        rospy.Subscriber("RCValues", Controller, self.callback_claw, queue_size=1)
        rospy.Subscriber("shutoff", Bool, self.callback_auto_shut_down, queue_size=1)
        self.homing_pub = rospy.Publisher("homing_info", HomingInfo, queue_size=1)

    def _read_byte(self):
        return struct.unpack('B', self.ser.read(1))[0]

if __name__ == '__main__':
    arduino = Arduino()
    arduino.start()
