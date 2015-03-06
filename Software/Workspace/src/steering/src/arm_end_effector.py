#!/usr/bin/python

from joystick_packages.msg import Joystick
from roboclaw import RoboClaw

import math
import rospy
import time
import traceback

class EndEffector:
    def __init__(self, topic = 'RCValues', node = 'RoboClaw'):
        self.min_battery = 119;

        self.joystick1 = Joystick()
        self.joystick2 = Joystick()
        self.node = node
        self.topic = topic

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber("joystick1", Joystick, self.callback1)
        rospy.Subscriber("joystick2", Joystick, self.callback2)
        self.rate = rospy.Rate(5)

        while not self.roboclaw_connect() and not rospy.is_shutdown():
            self.rate.sleep()

    def roboclaw_connect(self):
        try:
            print("Connecting to Roboclaws...")
            self.endAndStick = RoboClaw("/dev/roboclawfl")
            self.boom = RoboClaw("/dev/roboclawbl")
            return True
        except:
            return False

    def callback1(self, data):
        self.joystick1 = data

    def callback2(self, data):
        self.joystick2 = data

    def start(self):
        while not rospy.is_shutdown():
            if True or self.__check_batteries():
                try: 
                    self.run(self.endAndStick, self.joystick1.main_joy_x, 1)
                    self.run(self.endAndStick, self.joystick1.main_joy_y, 2)
                    self.run(self.boom, self.joystick2.main_joy_y, 1)
                except:
                    pass
            else:
                self.stop_run()
                self.stop_rotate()

            self.rate.sleep()

        self.stop_run()
        self.stop_rotate()

    def run(self, controller, target_speed, motor):
        target_speed = int((target_speed/90)*127)
        forward = [controller.M1Forward, controller.M2Forward]
        backward = [controller.M1Backward, controller.M2Backward]

        motorForward = forward[motor - 1]
        motorBackward = backward[motor - 1]

        try:
            if target_speed > 10:
                motorForward(target_speed)
            elif target_speed < -10:
                motorBackward(abs(target_speed))
            else:
                motorForward(0)
                motorBackward(0)
        except:
            self.stop_run()
            traceback.print_exc()

    def stop_run(self):
        try:
            self.endAndStick.M1Forward(0)
            self.endAndStick.M2Forward(0)
            self.boom.M1Forward(0)
        except:
            traceback.print_exc()

    def __check_batteries(self):
        try:
            if self.endAndStick.readmainbattery() < self.min_battery or self.boom.readmainbattery() < self.min_battery:
                return False
        except:
            pass

        return True

if __name__ == '__main__':
    args = ["RCValues", "RoboClaw"]
    endEffector = EndEffector(topic = args[0], node = args[1])
    endEffector.start()
