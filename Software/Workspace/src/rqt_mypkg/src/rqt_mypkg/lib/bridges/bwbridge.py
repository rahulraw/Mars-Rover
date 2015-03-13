import binascii
import cv2
import numpy as np
from cameras.msg import GrayImage

class InvalidBWImageException(Exception):
    pass

class BWBridge:
    def __init__(self):
        self._type = GrayImage

    def getType(self):
        return self._type

    def to_imgmsg(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        grayImage = GrayImage()
        if not len(img):
            raise InvalidBWImageExcapetion()
        grayImage.width = len(img[0])
        grayImage.image = []
        [[grayImage.image.append(value) for value in column] for column in img]

        return grayImage

    def from_imgmsg(self, grayImage):
        img = np.resize([int(binascii.hexlify(value), 16) for value in grayImage.image], (len(grayImage.image)/grayImage.width, grayImage.width))
        img = np.array(img, dtype=np.uint8)

        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
