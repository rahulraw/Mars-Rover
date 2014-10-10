#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from lib.messages import Messages
from lib.webcam import Webcam
from sensor_msgs.msg import Image

class Receiver:
    def __init__(self, topic = 'video_stream', node = 'receiver'):
        self.webcam = Webcam()
        self.topic = topic
        self.node = node
        self.bridge = CvBridge()

    def callback(self, video):
        image = None

        try:
            image = self.bridge.imgmsg_to_cv2(video, 'bgr8')
        except CVBridgeError, e:
            rospy.loginfo(Messages.cv_bridge_error %e) 

        self.webcam.show_image(image)
        rospy.loginfo(Messages.image_received %(video.width, video.height, video.header.stamp.to_sec(), rospy.get_time()))
        cv2.waitKey(5)

    def start(self):
        rospy.Subscriber(self.topic, Image, self.callback)
        rospy.init_node(self.node, anonymous=True)
        rospy.spin()
        
if __name__ == '__main__':
    receiver = Receiver()
    receiver.start()
