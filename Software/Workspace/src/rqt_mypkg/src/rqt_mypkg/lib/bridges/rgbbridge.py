import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class RGBBridge:
    def __init__(self):
        self._type = Image
        self.bridge = CvBridge()

    def getType(self):
        return self._type

    def to_imgmsg(self, image):
        return self.bridge.cv2_to_imgmsg(image, "bgr8")

    def from_imgmsg(self, image):
        return self.bridge.imgmsg_to_cv2(image, 'bgr8')
