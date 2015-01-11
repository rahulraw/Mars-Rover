import rospy
from std_msgs.msg import Float32MultiArray
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
        self.SafetyConstant = 0.5;
        self.running = True;

    ## rotation variables ##
        self.deg = 0;
        self.num_ticks = 70.368*self.deg;
        self.m1EncVal = 0;
        self.proportionalConstant = 0.014;
        self.integralConstant = 0;
        self.ErrThres = 500;
        self.running = True;

        self.motorController = RoboClaw("/dev/ttyACM1")
        #self.motorControllerFR = RoboClaw("/dev/ttyACM2")
        #self.motorControllerBL = RoboClaw("/dev/ttyACM3")
        #self.motorControllerBR = RoboClaw("/dev/ttyACM4")

    def callback(self, data):
        try:
            self.Elev =  data.data[0]; ## Order of which class method might be important.
            rospy.loginfo("The Elev value is: %s" % (data.data[0]))
            self.Thro = data.data[1];
            #self.num_ticks = 70.368*self.deg;
            rospy.loginfo("The Thro value is: %s" % (data.data[1]))
            
            self.spd = self.Elev;
            self.run(self.motorController);
            self.deg = self.Thro;
            self.num_ticks = 70.368*self.deg;
            self.rotate(self.motorController);

           #steering_calc.calc_all(self.Thro, self.Elev, False);
            
            #self.spd = steering_calc.velocity_left;
            #self.run(self.motorControllerFL);
            #self.run(self.motorControllerBL);
            #self.spd = steering_calc.velocity_right;
            #self.run(self.motorControllerFR);
            #self.run(self.motorControllerBR);           

            #self.deg = steering_calc.front_left_angle;
            #self.num_ticks = 70.368*self.deg;
            #self.rotate(self.motorControllerFL);

            #self.deg = steering_calc.front_right_angle;
            #self.num_ticks = 70.368*self.deg;
            #self.rotate(self.motorControllerFR);

            #self.deg = steering_calc.back_left_angle;
            #self.num_ticks = 70.368*self.deg;
            #self.rotate(self.motorControllerBL);

            #self.deg = steering_calc.back_right_angle;
            #self.num_ticks = 70.368*self.deg;
            #self.rotate(self.motorControllerBR);

        except:
            traceback.print_exc()
            rospy.loginfo("There has been an error")

    def start(self):
        self.motorController.sendcommand(128,21);
        #self.motorControllerFR.sendcommand(128,21);
        #self.motorControllerBL.sendcommand(128,21);
        #self.motorControllerBR.sendcommand(128,21);

        rcv = self.motorController.port.read(32)
        print(repr(rcv))
        #rcvFR = self.motorControllerFR.port.read(32)
        #print(repr(rcvFR))
        #rcvBL = self.motorControllerBL.port.read(32)
        #print(repr(rcvBL))
        #rcvBR = self.motorControllerBR.port.read(32)
        #print(repr(rcvBR))

        #steering_calculations = steering_calc(0, 0, False);

        #self.m1EncVal = self.motorController.readM1encoder()[0];
        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Float32MultiArray, self.callback)
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
    def rotate(self,motorController):
       self.m1EncVal = self.__convert_True_to_Mod__(motorController.readM1encoder()[0]);
       print("Mod Position is: ", self.m1EncVal);
       Err = self.num_ticks - self.m1EncVal;
       print('New Error: ', Err);
       Reset = Err;

       if Err > self.ErrThres:
           print('Position of Motor1: ', self.m1EncVal);
           motorValue = min(30, int((Err)*self.proportionalConstant + self.integralConstant*Reset))

       elif Err < -self.ErrThres:
           print('Position of Motor1: ', self.m1EncVal);
           motorValue = max(-30, int((Err)*self.proportionalConstant + self.integralConstant*Reset))
       else:
           motorValue = 0;
     
        #Error Checking 
       if abs(Err) > 180*(70.368):
           print "Error, max error exceeded.";
           motorValue = 0;
        
       if abs(self.m1EncVal) > 100*70.368:
           print "Error, 100 or -100 positioned reached";
           motorValue = 0;
       
       if motorValue > 0: 
            motorController.M1Forward(abs(motorValue));
       else: 
            motorController.M1Backward(abs(motorValue));
       print('Motor Value: ', motorValue);
       return

if __name__ == '__main__':
    args = ["RCValues", "RoboClaw"]
    carsteering = Steering(topic = args[0], node = args[1])
    carsteering.start()