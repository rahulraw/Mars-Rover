#!/usr/bin/python

from beeper import Beeper
from joystick_packages.msg import Controller
from roboclaw import RoboClaw
from steering_calc import SteeringCalc
from steering.msg import RoverInfo, HomingInfo

import math
import rospy
import serial
import struct
import time
import traceback

class Steering:
    def __init__(self, topic = 'RCValues', node = 'RoboClaw'):
        self.SAFETY_CONSTANT = 0.07

        self.joystick = Controller()
        self.homing = False
        self.node = node
        self.topic = topic

        self.errorThres = 35
        self.min_battery = 119
        self.proportionalConstant = 0.014

        self.beeper = Beeper(0.5, 1100)
        self.beeper.start()

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Controller, self.callback)
        rospy.Subscriber("homing_info", HomingInfo, self.homing_callback)
        self.info_pub = rospy.Publisher("RoverInfo", RoverInfo, queue_size = 10)
        self.rate = rospy.Rate(5)

        while not self.roboclaw_connect() and not rospy.is_shutdown():
            self.rate.sleep()

        self.mode = 1
        self.modes = ["Bicycle", "Strafe", "Turn On Self"]
        self.steering_calc = SteeringCalc(0, 0)
        self.steering_calculations = [self.steering_calc.bicycle, self.steering_calc.strafe, self.steering_calc.turn_onself]

        self.controllers = [self.controllerFL, self.controllerFR, self.controllerBL, self.controllerBR]
        self.roboclaw_set_param()
        self.velocity = [0, 0, 0, 0]
        self.angle = [0, 0, 0, 0]

        for controller in self.controllers:
            print(controller.readversion())

    def roboclaw_connect(self):
        try:
            print("Connecting to Roboclaws...")
            self.controllerFL = RoboClaw("/dev/roboclawfl", 1)
            self.controllerFR = RoboClaw("/dev/roboclawfr", -1)
            self.controllerBL = RoboClaw("/dev/roboclawbl", -1)
            self.controllerBR = RoboClaw("/dev/roboclawbr", 1)
            return True
        except:
            return False

    def homing_callback(self, data):
        if self.homing:
            homed = 0
            if data.front_left:
                self.controllerFL.M1Forward(0)
                homed += 1
            if data.front_right:
                self.controllerFR.M1Forward(0)
                homed += 1
            if data.back_left:
                self.controllerBL.M1Forward(0)
                homed += 1
            if data.back_right:
                self.controllerBR.M1Forward(0)
                homed += 1

            self.homing = False if homed == 1 else self.homing

    def callback(self, data):
        if not self.joystick.touch_button == data.touch_button:
            print("Killswitch {0}".format(data.touch_button))

        if not self.joystick.share == data.share and data.share == 1:
            self.roboclaw_connect()

        if data.options != self.joystick.options and data.options == 1:
            self.homing = True

        if data.mode != self.joystick.mode:
            print(self.modes[data.mode % 3])

        self.joystick = data

        if not self.joystick.touch_button:
            self.steering_calculations[self.joystick.mode % 3](data)
            self.velocity = [self.steering_calc.velocity_left, self.steering_calc.velocity_right, self.steering_calc.velocity_left, self.steering_calc.velocity_right]
            self.angle = [self.steering_calc.front_left_angle, self.steering_calc.front_right_angle, self.steering_calc.back_left_angle, self.steering_calc.back_right_angle]

            rover_info = RoverInfo()
            rover_info.front_left_angle = self.steering_calc.front_left_angle
            rover_info.front_right_angle = self.steering_calc.front_right_angle
            rover_info.back_left_angle = self.steering_calc.back_left_angle
            rover_info.back_right_angle = self.steering_calc.back_right_angle
            self.info_pub.publish(rover_info);

    def start(self):
        [controller.ResetEncoderCnts() for controller in self.controllers]
            
        while not rospy.is_shutdown():
            if not self.joystick.touch_button: # and self.__check_batteries():
                try: 
                    if (self.homing):
                        self.controllerFL.M1Backward(10)
                        self.controllerBR.M1Backward(10)
                        self.controllerFR.M1Forward(10)
                        self.controllerBL.M1Forward(10)
                        while self.homing and not rospy.is_shutdown():
                            self.rate.sleep()
                        self.finish_homing()
                    else:
                        [self.run(controller, velocity) for controller, velocity in zip(self.controllers, self.velocity)]
                        [self.rotate(controller, angle * 70.368) for controller, angle in zip(self.controllers, self.angle)]
                except Exception, e:
                    print(e)
                    pass
            else:
                self.stop_run()
                self.stop_rotate()

            if self.joystick.square:
                self.print_controller("Front Left", self.controllerFL, self.steering_calc.back_left_angle)
            if self.joystick.triangle:
                self.print_controller("Front Right", self.controllerFR, self.steering_calc.back_left_angle)
            if self.joystick.circle:
                self.print_controller("Back Right", self.controllerBR, self.steering_calc.back_right_angle)
            if self.joystick.x:
                self.print_controller("Back Left", self.controllerBL, self.steering_calc.back_left_angle)
            self.rate.sleep()

        self.stop_run()
        self.stop_rotate()

    def print_controller(self, name, controller, angle):
        try:
            print("# {0} #".format(name))
            print("Encoder: {0}".format(self.__convert_True_to_Mod__(controller.readM1encoder()[0])))
            print("Target: {0}".format(angle * 70.368))
            print("Main Battery: {0}".format(controller.readmainbattery()))
            print("Max Current. Motor 1: {0}".format(controller.max_current_motor_1))
            print("Max Current. Motor 2: {0}".format(controller.max_current_motor_2))
        except:
            pass

    def pidrun(self, controller, target_velocity):
        try:
            m2_encoder_value = controller.readM2speed()[0]
            error = target_velocity - m2_encoder_value
            motorValue = int((error) * self.proportionalConstant)

            if abs(error) <= self.errorThres or target_velocity > 150:
               motorValue = 0
            
            thresholdedMotorValue = 0 if abs(motorValue) < 4 else max(10, abs(motorValue))
            if motorValue > 0:
                controller.M1Forward(thresholdedMotorValue)
            else:
                controller.M1Backward(thresholdedMotorValue)
        except:
            self.stop_run()
            traceback.print_exc()

    def run(self, controller, target_speed):
        CONST_ACCEL = 50000

        try:
            if target_speed > 10 or target_speed < -10:
                controller.SetM2SpeedAccel(CONST_ACCEL, int(self.map(target_speed, -90, 90, -20000, 20000)))
            else:
                controller.SetM2SpeedAccel(CONST_ACCEL, 0)
        except:
            self.stop_run()
            traceback.print_exc()

    def rotate(self, controller, num_ticks):
        try:
            error = num_ticks - self.__convert_True_to_Mod__(controller.readM1encoder()[0]) + controller.offset
            motorValue = max(-20, min(20, int((error) * self.proportionalConstant)))

            if abs(error) <= self.errorThres or num_ticks > 180 * 70.368:
               motorValue = 0

            thresholdedMotorValue = 0 if abs(motorValue) < 4 else max(8, abs(motorValue))

            if motorValue > 0:
                controller.M1Forward(thresholdedMotorValue)
            else:
                controller.M1Backward(thresholdedMotorValue)
        except:
            self.stop_rotate()
            traceback.print_exc()

    def stop_run(self):
        try:
            for controller in self.controllers:
                controller.speed = 0

            [controller.M1Forward(0) for controller in self.controllers]
        except:
            traceback.print_exc()
            self.beeper.set(True)

    def stop_rotate(self):
        try:
            [controller.M2Forward(0) for controller in self.controllers]
        except:
            traceback.print_exc()
            self.beeper.set(True)

    def finish_homing(self):

        for controller in self.controllers:
            controller.offset = self.__convert_True_to_Mod__(controller.readM1encoder()[0]) + (90 * 70.368) * controller.rotation_orientation

    def __check_batteries(self):
        try:
            for controller in self.controllers:
                if controller.readmainbattery() < self.min_battery:
                    return False
        except:
            pass

        return True

    def __convert_True_to_Mod__(self, ticks):
        return ticks - 2**32 if ticks > 2**31 else ticks

    def roboclaw_set_param(self):
        #These values are experimentally determined
        P_CONST = int(1 * 65536)
        I_CONST = int(0.05 * 65536)
        D_CONST = int(0 * 65536)
        QPPS = 50000

        for controller in self.controllers:

            #set the constants for speed control
            controller.SetM2pidq(P_CONST, I_CONST, D_CONST, QPPS)

    def map(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

if __name__ == '__main__':
    args = ["RCValues", "RoboClaw"]
    carsteering = Steering(topic = args[0], node = args[1])
    carsteering.start()
