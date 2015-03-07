import serial
import rospy
from  time import sleep

class PC_arduino:
    def __init__(self):
        rospy.init_node('serial_bridge', anonymous=True)
        self.setupTopics()
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.rate = rospy.Rate(5)

    def communication(self):
        while not rospy.is_shutdown():
            if serialInWaiting() > 0:
                data = serial.readline()
                self.process_line(data)
                self.ser.write(self.response)
            self.rate.sleep()
    
    def process_line(self, data):
        #...
        #self.response = ...
        pass
        
    def setup_topics(self):
        rospy.Sub

