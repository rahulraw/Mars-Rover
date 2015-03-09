import serial
import rospy
from  time import sleep
from joystick_packages.msg import Controller

class Arduino:
    def __init__(self):
        rospy.init_node('serial_bridge', anonymous=True)
        self.setup_topics()
        self.ser = serial.Serial('/dev/ttyACM2', 9600)
        self.rate = rospy.Rate(5)
        self.controller = Controller();

    def callback(self, controller):
        if controller.left_joy_x != self.controller.left_joy_x:
            self.controller = controller

    def start(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.claw();

    def claw(self):
        self.ser.write(chr(3))
        self.ser.write(chr((self.controller.left_joy_x + 90) / 2))
        
    def setup_topics(self):
        rospy.Subscriber("RCValues", Controller, self.callback, queue_size=1)

if __name__ == '__main__':
    arduino = Arduino()
    arduino.start()
