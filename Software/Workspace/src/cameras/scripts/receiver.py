#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys

from lib.bridges.bwbridge import BWBridge
from lib.bridges.rgbbridge import RGBBridge
from lib.messages import Messages
from lib.webcam import Webcam
from config import Config

class Receiver:
    def __init__(self):
        self.webcam = Webcam(Config.scale)
        self.topic = rospy.get_param('~topic', 'video_stream')
        self.node = rospy.get_param('~node', 'receiver')
        if Config.gray:
            self.bridge = BWBridge()
        else:
            self.bridge = RGBBridge()

    def callback(self, video):
        image = None

        image = self.bridge.from_imgmsg(video)

        self.webcam.show_image(image)
        cv2.waitKey(5)

    def start(self):
        rospy.Subscriber(self.topic, self.bridge.getType(), self.callback)
        rospy.init_node(self.node, anonymous=True)
        rospy.spin()
        
if __name__ == '__main__':
    receiver = Receiver()
    receiver.start()
