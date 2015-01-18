import rospy
from std_msgs.msg import Int16MultiArray
import serial
import struct
import time
import math
import traceback
from roboclaw import RoboClaw
from steering_code import SteeringCalc

## Testing ROS communication with Roboclaw ##

class Steering:
    def __init__(self,topic = 'RCValues', node = 'RoboClaw'):
        self.topic = topic;
        self.node = node;
        self.Thro = 0;
        self.Elev = 0;

    ## drive variables ##
        self.spd = 0;
        self.SafetyConstant = 0.5
        self.running = True

    ## rotation variables ##
        self.deg = 0;
        self.num_ticks = 70.368*self.deg
        self.m1EncVal = 0;
        self.proportionalConstant = 0.014
        self.integralConstant = 0
        self.ErrThres = 500
        self.running = True

        self.motorControllerFL = RoboClaw("/dev/roboclawfl")
        self.motorControllerFR = RoboClaw("/dev/roboclawfr")
        self.motorControllerBL = RoboClaw("/dev/roboclawbl")
        self.motorControllerBR = RoboClaw("/dev/roboclawbr")

    def callback(self, data):
        try:
            if data.data[2]:
                self.stop()
            else:
                
                self.Elev =  data.data[0] 
                rospy.loginfo("The Elev value is: %s" % (data.data[1]))
                self.Thro = data.data[1]
                rospy.loginfo("The Thro value is: %s" % (data.data[1]))
                
                self.steering_calc.update_values(self.Thro, self.Elev, False, False)

                self.spd = self.steering_calc.velocity_left
                self.run(self.motorControllerFL)
                self.run(self.motorControllerBL)
                self.spd = self.steering_calc.velocity_right
                self.run(self.motorControllerFR)
                self.run(self.motorControllerBR)           

                self.deg = self.steering_calc.front_left_angle
                self.num_ticks = 70.368*self.deg
                print('\nFront Left:')
                self.rotate(self.motorControllerFL, True)

                self.deg = self.steering_calc.front_right_angle;
                self.num_ticks = 70.368*self.deg;
               
                print('\nFront Right:')
                self.rotate(self.motorControllerFR, True);

                self.deg = self.steering_calc.back_left_angle;
                self.num_ticks = 70.368*self.deg;
                print('\nBack Left:')
                self.rotate(self.motorControllerBL, True)

                self.deg = self.steering_calc.back_right_angle
                self.num_ticks = 70.368*self.deg;
                print('\nBack Right:')
                self.rotate(self.motorControllerBR,True)

        except:
            traceback.print_exc()
            rospy.loginfo("There has been an error")

    def start(self):
        self.motorControllerFL.sendcommand(128,21);
        self.motorControllerFR.sendcommand(128,21);
        self.motorControllerBL.sendcommand(128,21);
        self.motorControllerBR.sendcommand(128,21);

        rcvFL = self.motorControllerFL.port.read(32)
        print(repr(rcvFL))
        rcvFR = self.motorControllerFR.port.read(32)
        print(repr(rcvFR))
        rcvBL = self.motorControllerBL.port.read(32)
        print(repr(rcvBL))
        rcvBR = self.motorControllerBR.port.read(32)
        print(repr(rcvBR))

        self.steering_calc = SteeringCalc(0, 0, False, False);

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Int16MultiArray, self.callback)
        rospy.spin()

    def run(self, motorController):
        self.safetyConstant= 0.5;

        if (self.spd > 10):
            motorController.M2Forward(int(self.SafetyConstant*abs(self.spd)));

        elif (self.spd < -10):
            motorController.M2Backward(int(self.SafetyConstant*abs(self.spd)));
        else:
            motorController.M2Forward(0);
            motorController.M2Backward(0);
        return

    # Converts encoder value to valid ticks.
    def __convert_True_to_Mod__(self, ticks):
        if ticks > 2**31:
            return ticks - 2**32;
        else:
            return ticks;

    #True Position: Tick value that ranges between 0 and 4 billion.
    #Mod Position: Tick value that ranges between -90*70.368 and +90.368*70.368.
    def rotate(self, motorController, to_print):
       if to_print:
           print ("True self.m1EncVal: ", motorController.readM1encoder()[0]) 
       self.m1EncVal = self.__convert_True_to_Mod__(motorController.readM1encoder()[0]);
       if to_print:
           print ("Mod self.m1EncVal: ", self.m1EncVal) 
       Err = self.num_ticks - self.m1EncVal;
       if to_print:
           print ("Desired Ticks Position:  ", self.num_ticks)	
           print('New Position Error: ', Err);
       Reset = Err

       if Err >= self.ErrThres:
           #if to_print:
               #print('Position of Motor1: ', self.m1EncVal)
          motorValue = min(30, int((Err)*self.proportionalConstant + self.integralConstant*Reset))

       elif Err <= -self.ErrThres:
           #if to_print:
               #print('Position of Motor1: ', self.m1EncVal)
           motorValue = max(-30, int((Err)*self.proportionalConstant + self.integralConstant*Reset))
       else:
           if to_print:
               print("Error within threshold at: ", self.m1EncVal)
           motorValue = 0
           time.sleep(0)

        #Error Checking
       if self.num_ticks > 180*(70.368):
           motorValue = 0;

       #if abs(self.m1EncVal) > 100*70.368:
           #print "Error, 100deg or -100deg position reached";
           #motorValue = 0;

	#motor speed assignment
       if motorValue > 0:
            motorController.M1Forward(motorValue)
       else:
            motorController.M1Backward(abs(motorValue))
       print('Motor Value: ', motorValue)
       return

    def stop(self):
        self.motorControllerFL.M1Forward(0)
        self.motorControllerBL.M1Forward(0)
        self.motorControllerFR.M1Forward(0)
        self.motorControllerBR.M1Forward(0)

        self.motorControllerFL.M2Forward(0)
        self.motorControllerBL.M2Forward(0)
        self.motorControllerFR.M2Forward(0)
        self.motorControllerBR.M2Forward(0)

        self.motorControllerFL.ResetEncoderCnts()
        self.motorControllerBL.ResetEncoderCnts()
        self.motorControllerFR.ResetEncoderCnts()
        self.motorControllerBR.ResetEncoderCnts()



    def strafe(RC_input, velocity):    
        #Move wheels to same angle.
        self.deg = RC_input
        rotate(self.MotorControllerFL)
        rotate(self.MotorControllerFR)
        rotate(self.MotorControllerBL)
        rotate(self.MotorControllerBR)

        #Drive wheels at same speed.
        self.spd = velocity
        run(self.MotorControllerFL)
        run(self.MotorControllerFR)
        run(self.MotorControllerFL)
        run(self.MotorControllerBR)


    def startup():
        #Move BR & FR negative theta (M1Backward) until mag. proximity sensor.
        #Move BL & FL positive theta (M1Forward) until mag. proximity sensor.
        #Once mag. sensor is on:
            #BR: num_ticks = 90*(70.368), M1Forward 
            #FR: num_ticks = 90*(70.368), M1Backward
            #BL: num_ticks = 90*(70.368), M1Forward
            #FL: num_ticks = 90*(70.368), M1Backwar

        if self.alignedBR and self.alignedFR and self.alignedBL and self.alignedBR:
            ticksTo90 = 90*70.368;
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
