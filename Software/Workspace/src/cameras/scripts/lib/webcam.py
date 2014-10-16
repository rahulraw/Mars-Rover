import cv2
import numpy as np

class ImageNoneTypeException(Exception):
    pass

class Webcam:
    def __init__(self, scale = 1, camera_id = 1):
        self.scale = scale
        self.camera_id = camera_id
        self.playingVideo = True

    def start_capture(self): 
        self.capture = cv2.VideoCapture(self.camera_id)

    def read_image(self):
        _,image = self.capture.read()

        if image == None:
            raise ImageNoneTypeException()

        if self.scale != 1:
            image = cv2.resize(image, (0,0), fx = self.scale, fy = self.scale)

        return image

    def show_image(self, image):
        cv2.imshow('e2', image)

    def close(self):
        cv2.destroyAllWindows()
