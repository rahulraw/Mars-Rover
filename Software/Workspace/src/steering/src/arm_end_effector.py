#!/usr/bin/python

from joystick_packages.msg import Controller
from roboclaw import RoboClaw

import math
import rospy
import time
import traceback

class EndEffector:
    def __init__(self, topic = 'RCValues', node = 'RoboClaw'):
        self.min_battery = 119;

        self.joystick = Controller()
        self.node = node
        self.topic = topic

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Controller, self.callback)
        self.rate = rospy.Rate(5)

        while not self.roboclaw_connect() and not rospy.is_shutdown():
            self.rate.sleep()

    def roboclaw_connect(self):
        try:
            print("Connecting to Roboclaws...")
            self.armEndEffector = RoboClaw("/dev/ttyACM1")
            return True
        except:
            return False

    def callback(self, data):
        self.joystick = data

    def start(self):
        self.armEndEffector.ResetEncoderCnts()
            
        while not rospy.is_shutdown():
            if True or self.__check_batteries():
                try: 
                    self.run(self.armEndEffector, self.joystick.left_joy_y)
                except:
                    pass
            else:
                self.stop_run()
                self.stop_rotate()

            self.rate.sleep()

        self.stop_run()
        self.stop_rotate()

    def run(self, controller, target_speed):
        target_speed = int((target_speed/90)*127)
       
        try:
            if target_speed > 10:
                controller.M1Forward(target_speed)
            elif target_speed < -10:
                controller.M1Backward(abs(target_speed))
            else:
                controller.M1Forward(0)
                controller.M1Backward(0)
        except:
            self.stop_run()
            traceback.print_exc()

    def stop_run(self):
        try:
            self.armEndEffector.M1Forward(0)
        except:
            traceback.print_exc()

    def __check_batteries(self):
        try:
            if self.armEndEffector.readmainbattery() < self.min_battery:
                return False
        except:
            pass

        return True

if __name__ == '__main__':
    args = ["RCValues", "RoboClaw"]
    endEffector = EndEffector(topic = args[0], node = args[1])
    endEffector.start()
