import rospy
from std_msgs.msg import Float32MultiArray
import serial
import struct
import time
import math
import traceback
import roboclaw

## Testing ROS communication with Roboclaw ##

class steering:
    def __init__(self,topic = 'RCValues', node = 'RoboClaw'):
        
	self.topic = topic;
        self.node = node;	

	## drive variables ##        
	self.spd = 0;
        self.SafetyConstant = 0.5;
        self.running = True;
 
	## rotation variables ##       
        self.deg = 0;
        self.num_ticks = 70.368*self.deg;
        self.m1EncVal = 0;
        self.proportionalConstant = 0.1;
        self.integralConstant = 0;
    	self.ErrThres = 50;
        self.running = True;
        #self.ticksCount = self.ticksCount + num_ticks;	

    def callback(self, data):
       try:
           self.spd = data.data[0]; ## Order of which class method might be important.
           rospy.loginfo("The spd value is: %s" % (data.data[0]))
           self.deg = data.data[1];
	   self.num_ticks = 70.368*self.deg;
	   self.m1EncVal = roboclaw.readM1encoder()[0];
           rospy.loginfo("The direction value is: %s" % (data.data[1]))
           self.rotate();	   
	   self.run();
       except:
       	   traceback.print_exc()
           rospy.loginfo("There has been an error") 

    def start(self):
        roboclaw.port = serial.Serial("/dev/ttyACM2", baudrate=38400, timeout=1)

        roboclaw.sendcommand(128,21);
        rcv = roboclaw.port.read(32)
        print(repr(rcv))

	self.m1EncVal = roboclaw.readM1encoder()[0];	
	rospy.init_node(self.node, anonymous=True)       
	rospy.Subscriber(self.topic, Float32MultiArray, self.callback)	
	rospy.spin()
    
    def run(self):
	self.safetyConstant= 0.5;
	  
	if (self.spd > 0):
		roboclaw.M2Forward(int(self.SafetyConstant*abs(self.spd)));

	elif (self.spd < 0):
		roboclaw.M2Backward(int(self.SafetyConstant*abs(self.spd)));
	else:
		roboclaw.M2Forward(0);
		roboclaw.M2Backward(0);
	return
    
    # Converts encoder value to valid ticks.
    def __convert_True_to_Mod__(self, ticks):
	if ticks > 2**31:
		return ticks - 2**32;
	else:
		return ticks;    

    #True Position: Tick value that ranges between 0 and 4 billion.
    #Mod Position: Tick value that ranges between -90*70.368 and +90.368*70.368.
    def rotate(self):
        #while self.running:
           self.m1EncVal = self.__convert_True_to_Mod__(roboclaw.readM1encoder()[0]);
           # print("Mod Position is: ", self.m1EncVal);
           Err = self.num_ticks - self.m1EncVal;
           # print('New Error: ', Err);
           Reset = Err;
           motorValue = min(50, int(abs(Err)*self.proportionalConstant + self.integralConstant*Reset))
           print('Motor Value: ', motorValue)
           if Err > self.ErrThres:
                roboclaw.M1Forward(motorValue);
                print('Position of Motor1(Move Forward): ', self.m1EncVal);

           elif Err < -self.ErrThres:
	        roboclaw.M1Backward(motorValue);                
                print('Position of Motor1(Move Backward): ', self.m1EncVal);

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
    RoboClaw = steering(topic = args[0], node = args[1])
    RoboClaw.start()
