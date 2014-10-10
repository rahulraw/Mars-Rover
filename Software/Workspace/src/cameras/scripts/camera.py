#!/usr/bin/env python

import rospy
import traceback

from cv_bridge import CvBridge, CvBridgeError
from lib.messages import Messages
from lib.webcam import Webcam, ImageNoneTypeException
from sensor_msgs.msg import Image

class Camera:
    def __init__(self, topic = 'video_stream', node = 'camera'):
        self.webcam = Webcam(scale = 0.5, camera_id = 0)
        self.webcam.start_capture()
        self.topic = topic
        self.node = node
        self.bridge = CvBridge()

    def start(self):
        rospy.init_node(self.node, anonymous = True)

        self.pub = rospy.Publisher(self.topic, Image, queue_size = 10)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            image = None
            time = rospy.get_time()

            try:
                image = self.webcam.read_image()
            except ImageNoneTypeException, e:
                rospy.loginfo(Messages.image_none_type %(time, e))
                
            rospy.loginfo("Image published at %s" %time)

            try:
                self.pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            except CvBridgeError, e:
                rospy.loginfo(Messages.cv_bridge_error %e)

            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        camera = Camera()
        camera.start()
    except rospy.ROSInterruptException: pass
