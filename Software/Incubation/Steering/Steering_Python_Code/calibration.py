import rospy
import serial
import struct
import time
import math
import traceback
from std_msgs.msg import Bool
from roboclaw import RoboClaw

## Calibration code to align wheels on startup ##

class Calibration:
    def __init__(self):
        self.topic = 'calibration'
        self.node = 'Arduino'
        self.align_speed = 5
        self.align_msg = 0
        self.alignedFL = False
        self.alignedFR = False
        self.alignedBL = False
        self.alignedBR = False
        self.motorControllerFL = RoboClaw("/dev/roboclawfl")
        self.motorControllerFR = RoboClaw("/dev/roboclawfr")
        self.motorControllerBR = RoboClaw("/dev/roboclawbl")
        self.motorControllerBL = RoboClaw("/dev/roboclawbr")
        self.to_calibrate = True

    def start(self):
        while not rospy.is_shutdown():
            rospy.init_node(self.node, self.topic, self.callback )
            rospy.Subscriber(self.node, self.topic, self.callback)
            self.pub = rospy.Publisher(self.topic, Bool, que_size = 10)
            rospy.spin()

# Publish to calibrate
# call align_sequence

    def callback(self, data):
        align_msg = data
        self.alignedFL = data.data[0] 
        self.alignedFR = data.data[1]
        self.alignedBL = data.data[2] 
        self.alignedBR = data.data[3]
        rospy.loginfo("Message recieved ", data)

    def align_sequence(self, motor_controller):
        if (motor_controller == self.motorControllerFL and not self.alignedFL) or (motor_controller == self.motorControllerBR and not self.alignedBR):
            motor_controller.M1Forward(self.align_speed)
         
        if (motor_controller == self.motorControllerFR and not self.alignedFR) or (motor_controller == self.motorControllerBL and not self.alignedBR):
            motor_controller.M1Backward(self.align_speed)

        if self.alignedFR and self.alignedFL and self.alignedBR and self.alignedBL:
            self.to_calibrate = False 

        self.pub.publish(self.to_calibrate)

if __name__ == '__main__':
    calibration = Calibration()
    calibration.start()
