import rospy
from joystick_packages.msg import JoystickMsg
import serial
import struct
import time
import math
import traceback
from roboclaw import RoboClaw
from steering_calc import SteeringCalc

## Testing ROS communication with Roboclaw ##

class Steering:
    def __init__(self, topic = 'RCValues', node = 'RoboClaw'):
        self.topic = topic
        self.node = node

        ## drive variables ##
        self.SafetyConstant = 0.07

        ## rotation variables ##
        self.proportionalConstant = 0.014
        self.integralConstant = 0.0
        self.errorThres = 35
        self.steering_calc = SteeringCalc(0, 0)
        self.min_battery = 11.9
        self.speed = 0
        self.mode = 0
        self.integral = 0
        self.joystick = JoystickMsg()

        self.motorControllerFL = RoboClaw("/dev/roboclawfl")
        self.motorControllerFR = RoboClaw("/dev/roboclawfr")
        self.motorControllerBL = RoboClaw("/dev/roboclawbl")
        self.motorControllerBR = RoboClaw("/dev/roboclawbr")

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, JoystickMsg, self.callback)
        self.rate = rospy.Rate(5)

    def callback(self, data):
        self.joystick = data
        self.mode = (self.joystick.right_bumper - self.joystick.left_bumper) % 3
        if not self.joystick.killswitch:
            self.steering_calc.update_values(data.right_joy_x, data.left_joy_y)

    def start(self):
        # while not rospy.is_shutdown():
        #     rospy.spin()
        #     self.rate.sleep()
        # print(repr(self.motorControllerFL.readversion()))
        # print(repr(self.motorControllerFR.readversion()))
        # print(repr(self.motorControllerBL.readversion()))
        # print(repr(self.motorControllerBR.readversion()))

        self.motorControllerFL.ResetEncoderCnts()
        self.motorControllerFR.ResetEncoderCnts()
        self.motorControllerBL.ResetEncoderCnts()
        self.motorControllerBR.ResetEncoderCnts()

        while not rospy.is_shutdown():
            if not self.joystick.killswitch and self.check_batteries():
                self.run(self.motorControllerFL, self.steering_calc.velocity_left)
                self.run(self.motorControllerBL, self.steering_calc.velocity_left)
                self.run(self.motorControllerFR, self.steering_calc.velocity_right)
                self.run(self.motorControllerBR, self.steering_calc.velocity_right)

                self.rotate(self.motorControllerFL, self.steering_calc.front_left_angle * 70.368, 1)
                self.rotate(self.motorControllerFR, self.steering_calc.front_right_angle * 70.368, 2)
                self.rotate(self.motorControllerBL, self.steering_calc.back_left_angle * 70.368, 3)
                self.rotate(self.motorControllerBR, self.steering_calc.back_right_angle * 70.368, 4)

                try:
                    if self.joystick.square:
                        print("FL encoder: ", self.__convert_True_to_Mod__(self.motorControllerFL.readM1encoder()[0]))
                        print("FL target: ", self.steering_calc.front_left_angle * 70.368)
                        print("FL main battery: ", self.motorControllerFL.readmainbattery())
                    if self.joystick.triangle:
                        print("FR encoder: ", self.__convert_True_to_Mod__(self.motorControllerFR.readM1encoder()[0]))
                        print("FR target: ", self.steering_calc.front_right_angle * 70.368)
                        print("FR main battery: ", self.motorControllerFR.readmainbattery())
                    if self.joystick.circle:
                        print("BR encoder: ", self.__convert_True_to_Mod__(self.motorControllerBR.readM1encoder()[0]))
                        print("BR target: ", self.steering_calc.back_right_angle * 70.368)
                        print("BR main battery: ", self.motorControllerBR.readmainbattery())
                    if self.joystick.x:
                        print("BL encoder: ", self.__convert_True_to_Mod__(self.motorControllerBL.readM1encoder()[0]))
                        print("BL target: ", self.steering_calc.back_left_angle * 70.368)
                        print("BL main battery: ", self.motorControllerBL.readmainbattery())
                except:
                    pass
            else:
                self.stop_run()
                self.stop_rotate()
        
            self.rate.sleep()

        self.stop_run()
        self.stop_rotate()
        self.motorControllerFL.ResetEncoderCnts()
        self.motorControllerFR.ResetEncoderCnts()
        self.motorControllerBL.ResetEncoderCnts()
        self.motorControllerBR.ResetEncoderCnts()

    def run(self, motor_controller, target_speed):
        try:
            self.speed += int((target_speed - self.speed) * self.SafetyConstant)
            motorValue = self.speed
            if self.speed > 10:
                motor_controller.M2Forward(motorValue)
            elif self.speed < -10:
                motor_controller.M2Backward(abs(motorValue))
            else:
                motor_controller.M2Forward(0)
                motor_controller.M2Backward(0)
        except:
            self.stop_run()
            rospy.loginfo("Failure in run")
            traceback.print_exc()

    def check_batteries(self):
        try:
            return self.motorControllerFL.readmainbattery() > self.min_battery and self.motorControllerBL.readmainbattery() > self.min_battery and self.motorControllerFR.readmainbattery() > self.min_battery and self.motorControllerBR.readmainbattery() > self.min_battery
        except:
            return True

    # Converts encoder value to valid ticks.
    def __convert_True_to_Mod__(self, ticks):
        if ticks > 2**31:
            return ticks - 2**32
        else:
            return ticks

    # True Position: Tick value that ranges between 0 and 4 billion.
    # Mod Position: Tick value that ranges between -90*70.368 and +90*70.368.
    def rotate(self, motor_controller, num_ticks, motor_controller_index = 0):
        try:
            m1_encoder_value = self.__convert_True_to_Mod__(motor_controller.readM1encoder()[0])
            error = num_ticks - m1_encoder_value
            self.integral = self.integral + error
            motorValue = max(-20, min(20, int((error) * self.proportionalConstant + self.integral * self.integralConstant)))

            if (motor_controller_index == -2):
                print("Error: ", error)
                print("Target: ", num_ticks)
                print("Encoder: ", m1_encoder_value)
                print("Integral: ", self.integral)
                print("Motor Value: ", motorValue)

            if abs(error) <= self.errorThres or num_ticks > 180 * 70.368:
               motorValue = 0

            thresholdedMotorValue = 0 if abs(motorValue) < 4 else max(8, abs(motorValue))
            if motorValue > 0:
                motor_controller.M1Forward(thresholdedMotorValue)
            else:
                motor_controller.M1Backward(thresholdedMotorValue)
        except:
            self.stop_rotate()
            rospy.loginfo("Failure in rotate")
            traceback.print_exc()

    def stop_run(self):
        rospy.loginfo("Stopped running")
        self.speed = 0
        self.motorControllerFL.M1Forward(0)
        self.motorControllerBL.M1Forward(0)
        self.motorControllerFR.M1Forward(0)
        self.motorControllerBR.M1Forward(0)

    def stop_rotate(self):
        rospy.loginfo("Stopped rotating")
        self.motorControllerFL.M2Forward(0)
        self.motorControllerBL.M2Forward(0)
        self.motorControllerFR.M2Forward(0)
        self.motorControllerBR.M2Forward(0)

    def strafe(self, RC_input, velocity):    
        target_ticks = RC_input * 70.368
        self.rotate(self.MotorControllerFL, target_ticks)
        self.rotate(self.MotorControllerFR, target_ticks)
        self.rotate(self.MotorControllerBL, target_ticks)
        self.rotate(self.MotorControllerBR, target_ticks)

        target_speed = velocity
        self.run(self.MotorControllerFL, target_speed)
        self.run(self.MotorControllerFR, target_speed)
        self.run(self.MotorControllerFL, target_speed)
        self.run(self.MotorControllerBR, target_speed)

    def startup():
        #Move BR & FR negative theta (M1Backward) until mag. proximity sensor.
        #Move BL & FL positive theta (M1Forward) until mag. proximity sensor.
        #Once mag. sensor is on:
            #BR: num_ticks = 90*(70.368), M1Forward 
            #FR: num_ticks = 90*(70.368), M1Backward
            #BL: num_ticks = 90*(70.368), M1Forward
            #FL: num_ticks = 90*(70.368), M1Backwar

        if self.alignedBR and self.alignedFR and self.alignedBL and self.alignedBR:
            ticksTo90 = 90*70.368
            self.motorControllerBR.M1Forward(ticksto90)
            self.motorControllerFR.M1Backward(ticksto90)
            self.motorControllerBL.M1Forward(ticksto90)
            self.motorControllerFL.M1Backward(ticksto90)

        
        else:
            if self.alignedBR:
                self.motorControllerBR.M1Backward(0)
            else:
                self.motorControllerBR.M1Backward(10)

            if self.alignedFR:
                self.motorControllerBR.M1Backward(0)
            else:
                self.motorControllerBR.M1Backward(10)
            
            if self.alignedBL:
                self.motorControllerBR.M1Forward(0)
            else:
                self.motorControllerBR.M1Forward(10)
            
            if self.alignedFL:
                self.motorControllerBR.M1Forward(0)
            else:
                self.motorControllerBR.M1Forward(10)

            rospy.spin()
    

if __name__ == '__main__':
    args = ["RCValues", "RoboClaw"]
    carsteering = Steering(topic = args[0], node = args[1])
    carsteering.start()
