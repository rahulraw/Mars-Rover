#!/usr/bin/env python

import rospy
import sys
import traceback

from lib.bridges.bwbridge import BWBridge
from lib.bridges.rgbbridge import RGBBridge
from lib.webcam import Webcam, ImageNoneTypeException
from cameras.msg import GrayImage
from config import Config

class Camera:
    def __init__(self):
        self.topic = rospy.get_param('~topic', 'video_stream')
        self.node = rospy.get_param('~node', 'camera') 
        self.camera_id = int(rospy.get_param('~camera_id', 1))
        self.webcam = Webcam(Config.scale, self.camera_id)
        self.webcam.start_capture()
        if Config.gray:
            self.bridge = BWBridge()
        else:
            self.bridge = RGBBridge()

    def start(self):
        rospy.init_node(self.node, anonymous = True)

        self.pub = rospy.Publisher(self.topic, self.bridge.getType(), queue_size = 10)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            image = None
            time = rospy.get_time()

            try:
                image = self.webcam.read_image()
            except ImageNoneTypeException, e:
                rospy.loginfo(Messages.image_none_type %(time, e))
                
            rospy.loginfo("Image published at %s" %time)

            image = self.bridge.to_imgmsg(image)
            self.pub.publish(image)

            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        camera = Camera()
        camera.start()
    except rospy.ROSInterruptException: pass
