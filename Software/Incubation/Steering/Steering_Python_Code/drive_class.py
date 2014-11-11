import rospy
from std_msgs.msg import Float32MultiArray
import serial
import struct
import time
import math

import roboclaw

## Testing ROS communication with Roboclaw ##

class drive:
    def __init__(self,topic = 'RCValues', node = 'RoboClaw'):
        self.spd = 0;
        self.topic = topic;
        self.node = node;
        self.SafetyConstant = 0.5;
        self.running = True;
        
    def callback(self, data):
       try:
           self.spd = data.data[0];
           rospy.loginfo("The spd value is: %s" % (data.data[0]))
	   self.run();
       except:
           rospy.loginfo("There has been an error") 

    def start(self):
        roboclaw.port = serial.Serial("/dev/ttyACM1", baudrate=38400, timeout=1)

        roboclaw.sendcommand(128,21);
        rcv = roboclaw.port.read(32)
        print(repr(rcv))

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
		roboclaw.M1Forward(0);
		roboclaw.M1Backward(0);
	return


if __name__ == '__main__':
    args = ["RCValues", "RoboClaw"]
    RoboClaw = drive(topic = args[0], node = args[1])
    RoboClaw.start()

