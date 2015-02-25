#!/usr/bin/python

import os
import rospy
import threading
import time

from std_msgs.msg import Bool

class AutoShutdown(threading.Thread):
    """
        Check time to send shutdown message
    """
    def __init__(self, time):
        threading.Thread.__init__(self)
        self.updateTime()
        self.time = time
        self.pub_shutoff = rospy.Publisher('shutoff', Bool, queue_size = 1)

    def updateTime(self):
        self.current_time = time.time()

    def run(self):
        while not rospy.is_shutdown():
            if time.time() - self.current_time > self.time:
                shutoff = Bool()
                shutoff.data = True
                self.pub_shutoff.publish(shutoff)
            sleep(1)
