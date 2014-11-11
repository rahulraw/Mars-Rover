import rospy
from std_msgs.msg import Float32MultiArray
import serial
import struct
import time
import math

import roboclaw

class Rotation:
    def __init__(self, topic = 'RCValues', node = 'RoboClaw'):
        self.deg = 0;
        self.topic = topic;
        self.node = node;
        self.num_ticks = 70.368*self.deg;
        self.m1EncVal = 0;
        self.proportionalConstant = 0.8;
        self.integralConstant = 0;
    	self.ErrThres = 500;
        self.running = True;
        #self.ticksCount = self.ticksCount + num_ticks;

    def callback(self, data):
       try:
           self.deg = data.data[1];
	   self.num_ticks = 70.368*self.deg;
           rospy.loginfo("The direction value is: %s" % (data.data[1]))
	   self.running = True;
           self.rotate();
       except:
           self.running = False;
	   rospy.loginfo("There has been an error") 

    def start(self):
        roboclaw.port = serial.Serial("/dev/ttyACM1", baudrate=38400, timeout=1)

        roboclaw.sendcommand(128,21);
        rcv = roboclaw.port.read(32)
        print(repr(rcv))

	self.m1EncVal = roboclaw.readM1encoder()[0];
        rospy.Subscriber(self.topic, Float32MultiArray, self.callback)
        rospy.init_node(self.node, anonymous=True)
        rospy.spin()
	

    # Converts encoder value to valid ticks.
    def __convert_True_to_Mod__(self, ticks):
	if ticks > 2**31:
		return ticks - 2**31;
	else:
		return ticks;

    #True Position: Tick value that ranges between 0 and 4 billion.
    #Mod Position: Tick value that ranges between -90*70.368 and +90.368*70.368.
    def rotate(self):
        #while self.running:
           self.m1EncVal = self.__convert_True_to_Mod__(roboclaw.readM1encoder()[0]);
           print("Mod Position is: ", self.m1EncVal);
           Err = self.num_ticks - self.m1EncVal;
           print('New Error: ', Err);
           Reset = Err;
           if Err > self.ErrThres:
                roboclaw.M1Forward(int((Err*self.proportionalConstant + self.integralConstant*Reset)%50));
                #try:
                    #m1EncVal_new = self.__convert_True_to_Mod__(roboclaw.readM1encoder()[0]);
                #except:
                    #pass
                print('Position of Motor1(Move Forward): ', m1EncVal);

           elif Err < -self.ErrThres:
	        roboclaw.M1Backward(int((Err*self.proportionalConstant + self.integralConstant*Reset)%50));                
		#try:
                    #m1EncVal_new = self.__convert_True_to_Mod__(roboclaw.readM1encoder()[0]);
                #except:
                    #pass
                print('Position of Motor1(Move Backward): ',m1EncVal);

	   else:
		roboclaw.M1Forward(0);	   

           #Err = self.num_ticks - m1EncVal_new;
           #Reset = Reset + Err
           #print('New Error: ', Err);
      
        #roboclaw.M1Forward(0);
        #roboclaw.M1Backward(0);
	   return   

if __name__ == '__main__':
    args = ["RCValues", "RoboClaw"]
    RoboClaw = Rotation(topic = args[0], node = args[1])
    RoboClaw.start()
