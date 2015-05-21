#!/usr/bin/python

from joystick_packages.msg import Joystick, Controller
from roboclaw import RoboClaw
from jrk import Jrk

import math
import rospy
import time
import traceback

class ArmControl:
    def __init__(self):
        self.min_battery = 119

        self.joystick1 = Joystick()
        self.joystick2 = Joystick()

        rospy.init_node('ArmControl', anonymous=True)
        self.rate = rospy.Rate(5)

        while not self.motor_controller_connect() and not rospy.is_shutdown():
            self.rate.sleep()

        print("connected")
        rospy.Subscriber(rospy.get_param('~joystick1_topic','joystick1'), Joystick, self.callback1)
        rospy.Subscriber(rospy.get_param('~joystick2_topic','joystick2'), Joystick, self.callback2)


    def motor_controller_connect(self):
        try:
            print("Connecting to motor controllers...")
            self.boom = Jrk(rospy.get_param('~boom_id','/dev/boom'))
            self.stick = Jrk(rospy.get_param('~stick_id','/dev/stick'))
            # self.endAndYaw = RoboClaw(rospy.get_param('~endAndYaw_id','/dev/endAndYaw'))
            return True
        except:
            return False

    def get_direction(self, value):
        return int(0 if not value else value / abs(value))

    def callback(self, data):
        self.boom.set_direction(self.get_direction(data.left_joy_y))
        self.stick.set_direction(self.get_direction(data.right_joy_y))

    def callback1(self, data):
        self.boom.set_direction(self.get_direction(data.main_joy_y))
        self.joystick1 = data

    def callback2(self, data):
        self.stick.set_direction(self.get_direction(data.main_joy_y))
        self.joystick2 = data

    def start(self):
        self.boom.getErrorFlagsHalting()
        self.stick.getErrorFlagsHalting()

        while not rospy.is_shutdown():
            if True or self.__check_batteries():
                try: 
                    self.boom.run()
                    self.stick.run()

                    # Joystick 1 controls motor 2 which is the yaw motor
                    # self.run(self.endAndYaw, self.joystick2.main_joy_x, 2)

                    # Joystick 2 controls motor 1 which is the end effector 
                    # self.run(self.endAndYaw, self.joystick1.main_joy_x, 1)
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
            pass
            # self.endAndYaw.M1Forward(0)
            # self.endAndYaw.M2Forward(0)
        except:
            traceback.print_exc()

    def __check_batteries(self):
        try:
            return True
            if self.endAndYaw.readmainbattery() < self.min_battery:
                return False
        except:
            pass

        return True

if __name__ == '__main__':
    armControl = ArmControl()
    armControl.start()
