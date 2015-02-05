#!/usr/bin/python

import os
import rospy
import threading
import time

class Beeper(threading.Thread):
    """
        Can choose to make a beeping noise. Must have
        sox installed as a pre-requisite.
    """
    def __init__(self, duration, frequency):
        threading.Thread.__init__(self)

        self.duration = duration
        self.frequency = frequency
        self.exit = False
        self.on = False

    def set(self, on):
        self.on = on

    def exit(self):
        self.exit = True

    def run(self):
        while not rospy.is_shutdown():
            if self.on:
                os.system('play --no-show-progress --null --channels 1 synth %s sine %f' % (self.duration, self.frequency))
            time.sleep(self.duration)
