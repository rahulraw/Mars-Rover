#!/usr/bin/python
import serial
import struct
import rospy
from  time import sleep
from std_msgs.msg import Bool
from joystick_packages.msg import Controller
from steering.msg import HomingInfo
from rqt_mypkg.msg import CameraMountInfo

class Arduino:
    def __init__(self):
        self.ACCEPT_SERIAL = 255
        self.SEND_SERIAL = 254
        rospy.init_node("serial_node", anonymous=True)
        self.serial_port = rospy.get_param('~serial_port', '/dev/arduino')
        self.setup_topics()
        self.ser = serial.Serial(self.serial_port, 9600)
        self.rate = rospy.Rate(10)
        self.controller = Controller()
        self.send_manipulator = False
        self.send_camera_mount_info = False
        self.camera_mount_info = CameraMountInfo()
        self.shut_down = False
        self.send_shut_down = False
        self.message_length = 0
        self.messages = []
        self.publish_nodes = [self.homing]

    def callback_manipulator(self, controller):
        if controller.killswitch != self.controller.killswitch:
            shut_down = Bool()
            shut_down.data = self.controller.killswitch == 1
            self.callback_auto_shut_down(shut_down)
        self.controller = controller
        self.send_manipulator = True

    def callback_auto_shut_down(self, shut_down):
        self.send_shut_down = self.shut_down != shut_down.data
        self.shut_down = shut_down.data

    def callback_camera_mount_info(self, camera_mount_info):
        print(camera_mount_info)
        self.camera_mount_info = camera_mount_info
        self.send_camera_mount_info = True

    def start(self):
        while not rospy.is_shutdown():
            msg = self._read_byte()

            if (msg == self.ACCEPT_SERIAL):
                self.manipulator()
                self.auto_shut_down()
                self.camera_mount_control()
                self.send()
            elif (msg == self.SEND_SERIAL):
                topic = self._read_byte()
                length = self._read_byte()
                data = [self._read_byte() for i in range(length)]
                self.publish_nodes[topic - 1](data)
            else:
                print(msg)
                # Data needs to be passed into our other message

    def manipulator(self):
        if self.send_manipulator and self.controller.left_trigger == 0:
            self.message_length += 1
            self.messages.append(chr(1))
            self.messages.append(chr((self.controller.left_joy_x + 90) / 2))
            self.messages.append(chr((self.controller.right_joy_y + 90) / 2))
            self.messages.append(chr((self.controller.right_joy_x + 90) / 2))
            self.send_manipulator = False

    def auto_shut_down(self):
        if self.send_shut_down:
            self.message_length += 1
            self.messages.append(chr(2))
            self.messages.append(chr(1 if self.shut_down else 0))
            self.send_shut_down = False

    def camera_mount_control(self):
        if self.send_camera_mount_info:
            self.message_length += 1
            self.messages.append(chr(3))
            self.messages.append(chr(self.camera_mount_info.x_pos))
            self.messages.append(chr(self.camera_mount_info.y_pos))
            self.messages.append(chr(self.camera_mount_info.zoom))
            self.send_camera_mount_info = False

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
        rospy.Subscriber(rospy.get_param('~controller_topic', 'RCValues'), Controller, self.callback_manipulator, queue_size=1)
        rospy.Subscriber(rospy.get_param('~autoshutdown_topic', 'shutoff'), Bool, self.callback_auto_shut_down, queue_size=1)
        rospy.Subscriber(rospy.get_param('~cameramount_topic' , 'CameraMountInfo'), CameraMountInfo, self.callback_camera_mount_info, queue_size=10)
        self.homing_pub = rospy.Publisher(rospy.get_param('~homing_topic', 'homing_info'), HomingInfo, queue_size=1)

    def _read_byte(self):
        return struct.unpack('B', self.ser.read(1))[0]

if __name__ == '__main__':
    arduino = Arduino()
    arduino.start()
