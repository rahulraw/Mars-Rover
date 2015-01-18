import rospy
from std_msgs.msg import Int16MultiArray
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
        self.SafetyConstant = 0.5

        ## rotation variables ##
        self.proportionalConstant = 0.014
        self.integralConstant = 0
        self.errorThres = 500
        self.steering_calc = SteeringCalc(0, 0)

        self.motorControllerFL = RoboClaw("/dev/roboclawfl")
        self.motorControllerFR = RoboClaw("/dev/roboclawfr")
        self.motorControllerBL = RoboClaw("/dev/roboclawbl")
        self.motorControllerBR = RoboClaw("/dev/roboclawbr")

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Int16MultiArray, self.callback)
        self.rate = rospy.Rate(20)

    def callback(self, data):
        try:
            if data.data[2]:
                self.stop()
            else:
                elev = data.data[0] 
                thro = data.data[1]
                
                rospy.loginfo("The Thro value is: %s" % (thro))
                rospy.loginfo("The Elev value is: %s" % (elev)

                self.steering_calc.update_values(thro, elev)

        except:
            traceback.print_exc()
            rospy.loginfo("There has been an error")

    def start(self):
        self.motorControllerFL.sendcommand(128,21)
        self.motorControllerFR.sendcommand(128,21)
        self.motorControllerBL.sendcommand(128,21)
        self.motorControllerBR.sendcommand(128,21)

        rcvFL = self.motorControllerFL.port.read(32)
        print(repr(rcvFL))

        rcvFR = self.motorControllerFR.port.read(32)
        print(repr(rcvFR))

        rcvBL = self.motorControllerBL.port.read(32)
        print(repr(rcvBL))

        rcvBR = self.motorControllerBR.port.read(32)
        print(repr(rcvBR))

        while not rospy.is_shutdown():
            rospy.spinOnce()

            self.run(self.motorControllerFL, self.steering_calc.velocity_left)
            self.run(self.motorControllerBL, self.steering_calc.velocity_left)
            self.run(self.motorControllerFR, self.steering_calc.velocity_right)
            self.run(self.motorControllerBR, self.steering_calc.velocity_right)

            self.rotate(self.motorControllerFL, self.steering_calc.front_left_angle * 70.368)
            self.rotate(self.motorControllerFR, self.steering_calc.front_right_angle * 70.368)
            self.rotate(self.motorControllerBL, self.steering_calc.back_left_angle * 70.368)
            self.rotate(self.motorControllerBR, self.steering_calc.back_right_angle * 70.368)
            
            self.rate.sleep()

    def run(self, motor_controller, target_speed):
        self.safetyConstant = 0.5
        speed = int(self.SafetyConstant * abs(target_speed))
        forward_speed = speed if target_speed > 10 else 0
        backward_speed = speed if target_speed < -10 else 0
        motor_controller.M2Forward(forward_speed)
        motor_controller.M2Backward(backward_speed)

    # Converts encoder value to valid ticks.
    def __convert_True_to_Mod__(self, ticks):
        if ticks > 2**31:
            return ticks - 2**32
        else:
            return ticks

    # True Position: Tick value that ranges between 0 and 4 billion.
    # Mod Position: Tick value that ranges between -90*70.368 and +90*70.368.
    def rotate(self, motor_controller, num_ticks):
       m1_encoder_value = self.__convert_True_to_Mod__(motor_controller.readM1encoder()[0])
       error = num_ticks - m1_encoder_value
       integral += error

       motorValue = max(-20, min(20, int((error) * self.proportionalConstant + self.integralConstant * integral)))
       if abs(error) <= self.errorThres or num_ticks > 180 * 70.368:
           motorValue = 0

       if motorValue > 0:
            motor_controller.M1Forward(motorValue)
       else:
            motor_controller.M1Backward(abs(motorValue))

    def stop(self):
        self.motorControllerFL.M1Forward(0)
        self.motorControllerBL.M1Forward(0)
        self.motorControllerFR.M1Forward(0)
        self.motorControllerBR.M1Forward(0)

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
