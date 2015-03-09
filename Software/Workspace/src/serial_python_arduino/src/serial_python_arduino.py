import serial
import rospy
from  time import sleep
from joystick_packages.msg import Controller

class Arduino:
    def __init__(self):
        rospy.init_node('serial_bridge', anonymous=True)
        self.setup_topics()
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.rate = rospy.Rate(10)
        self.controller = Controller();

    def callback(self, controller):
        if controller.left_joy_x != self.controller.left_joy_x:
            self.controller = controller

    def start(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.claw();

    def claw(self):
        print(self.controller.left_joy_x);
        self.ser.write(chr(1))
        self.ser.write(chr((self.controller.left_joy_x + 90) / 2))
        
    def setup_topics(self):
        rospy.Subscriber("RCValues", Controller, self.callback, queue_size=1)

if __name__ == '__main__':
    arduino = Arduino()
    arduino.start()
