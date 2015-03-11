#!/usr/bin/env python

import cv2
import cv
import numpy as np
import rospy
import sys

from lib.bridges.bwbridge import BWBridge
from lib.bridges.rgbbridge import RGBBridge
from lib.messages import Messages
from lib.webcam import Webcam
from config import Config

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *

try:  
    from PyQt4.QtCore import QString  
except ImportError:  
    # we are using Python3 so QString is not defined  
    QString = str 

class camThread(QtCore.QThread):
    def __init__(self):
        QtCore.QThread.__init__(self)
        self.topic = rospy.get_param('~topic', 'video_stream')
        self.node = rospy.get_param('~node', 'receiver')
        if Config.gray:
            self.bridge = BWBridge()
        else:
            self.bridge = RGBBridge()

    def run(self):
        print("cam thread run")
        rospy.Subscriber(self.topic, self.bridge.getType(), self.callback)
        
    def callback(self, video):
        self.cv_image = None

        self.cv_image = self.bridge.from_imgmsg(video)
        
        if self.cv_image != None:
            # Notice the dimensions.
            height, width, bytesPerComponent = self.cv_image.shape
            bytesPerLine = 3 * width;
            # Convert to RGB for QImage.
            cv2.cvtColor(self.cv_image, cv.CV_BGR2RGB, self.cv_image)

            self.image = QImage(self.cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)

        self.emit(QtCore.SIGNAL('start(QImage)'), self.image)
        
class cameraWidget(QtGui.QWidget):
    
    def __init__(self, parent=None):
        super(cameraWidget, self).__init__(parent)
        
        self.webcam = Webcam(Config.scale)
        self.topic = rospy.get_param('~topic', 'video_stream')
        self.node = rospy.get_param('~node', 'receiver')
        if Config.gray:
            self.bridge = BWBridge()
        else:
            self.bridge = RGBBridge()
            
        #rospy.init_node(self.node, anonymous=True)
        
        self.cam_xpos = 0;
        self.cam_ypos = 0;
        self.cam_zpos = 0; 
        
        self.buttonLayout = QtGui.QGridLayout()
        # self.buttonLayout.setMargin(0)

        self.camControlArea = QtGui.QScrollArea()
        # self.camControlArea.setWidgetResizable(True)
        self.camControlArea.setLayout(self.buttonLayout)
        self.camControlArea.setFixedSize(500,200)
        
        self.createLabels()
        self.createTextBoxes()
        self.createScrollbars()
        self.createButtons() 	
        self.createControls()
        self.cameraWindow = QtGui.QLabel(self)
        self.cameraWindow.setScaledContents(True)
        self.cameraWindow.setFixedSize(500,400)
        
        self.cameraBox = QtGui.QVBoxLayout(self)
        self.cameraBox.addWidget(self.cameraWindow)
        self.cameraBox.addWidget(self.camControlArea)
        # self.cameraBox.addLayout(self.buttonLayout)
        
        self.setLayout(self.cameraBox)
        self.startCamera()

    def startCamera(self):
        self.cameraThread = camThread()
        self.connect(self.cameraThread, QtCore.SIGNAL('start(QImage)'), self.start, Qt.QueuedConnection)
        self.cameraThread.start()
        
    def setImage(self, pixmap):
        self.cameraWindow.setPixmap(pixmap)
        
    def start(self, cv_image):
        pixmap = QPixmap.fromImage(cv_image)
        self.setImage(pixmap)

    def createLabels(self):
        self.label_cam_zoom_in = QtGui.QLabel("+")
        self.label_cam_zoom_out = QtGui.QLabel("-")
        self.label_cam_zoom = QtGui.QLabel("zoom")
        self.label_cam_xpos = QtGui.QLabel("x pos")
        self.label_cam_ypos = QtGui.QLabel("y pos")

        self.label_cam_zoom_in.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.label_cam_zoom_out.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.label_cam_zoom.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        self.label_cam_xpos.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        self.label_cam_ypos.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)

        self.buttonLayout.addWidget(self.label_cam_zoom_in, 2, 3, 2, 1)
        self.buttonLayout.addWidget(self.label_cam_zoom_out, 2, 1, 2, 1)
        self.buttonLayout.addWidget(self.label_cam_zoom, 4, 2, 1, 1)
        self.buttonLayout.addWidget(self.label_cam_xpos, 4, 0, 1, 2)
        self.buttonLayout.addWidget(self.label_cam_ypos, 4, 3, 1, 2)

    def createTextBoxes(self):
        self.tb_cam_xpos = QtGui.QLineEdit()
        self.tb_cam_ypos = QtGui.QLineEdit()
        self.tb_cam_zpos = QtGui.QLineEdit()
        
        self.tb_cam_xpos.setReadOnly(True)
        self.tb_cam_ypos.setReadOnly(True)
        self.tb_cam_zpos.setReadOnly(True)
        
        self.tb_cam_xpos.setText(QString("0"))
        self.tb_cam_ypos.setText(QString("0"))
        self.tb_cam_zpos.setText(QString("0"))

    def createScrollbars(self):
        self.scrollbar_cam_zoom = QtGui.QScrollBar()
        self.scrollbar_cam_zoom.setOrientation(QtCore.Qt.Horizontal)
        self.scrollbar_cam_zoom.setMinimum(0)
        self.scrollbar_cam_zoom.setMaximum(100)
        self.scrollbar_cam_zoom.connect(self.scrollbar_cam_zoom, QtCore.SIGNAL("sliderMoved(int)"), self._cam_slider_moved)
        
    def createButtons(self):
        self.pb_cam_up = QtGui.QPushButton("Up")
        self.pb_cam_down = QtGui.QPushButton("Down")
        self.pb_cam_left = QtGui.QPushButton("Left")
        self.pb_cam_right = QtGui.QPushButton("Right")
        self.pb_cam_up.clicked[bool].connect(self._handle_pb_cam_up_clicked)
        self.pb_cam_down.clicked[bool].connect(self._handle_pb_cam_down_clicked)
        self.pb_cam_left.clicked[bool].connect(self._handle_pb_cam_left_clicked)
        self.pb_cam_right.clicked[bool].connect(self._handle_pb_cam_right_clicked)

    def createControls(self):
        self.buttonLayout.addWidget(self.label_cam_zoom_in, 2, 3, 2, 1)
        self.buttonLayout.addWidget(self.label_cam_zoom_out, 2, 1, 2, 1)
        self.buttonLayout.addWidget(self.label_cam_zoom, 4, 2, 1, 1)
        self.buttonLayout.addWidget(self.label_cam_xpos, 4, 0, 1, 2)
        self.buttonLayout.addWidget(self.label_cam_ypos, 4, 3, 1, 2)
        self.buttonLayout.addWidget(self.tb_cam_xpos, 5, 0, 1, 2)
        self.buttonLayout.addWidget(self.tb_cam_ypos, 5, 3, 1, 2)
        self.buttonLayout.addWidget(self.tb_cam_zpos, 5, 2, 1, 1)
        self.buttonLayout.addWidget(self.pb_cam_up, 0, 2, 1, 1)
        self.buttonLayout.addWidget(self.pb_cam_down, 3, 2, 1, 1)
        self.buttonLayout.addWidget(self.pb_cam_left, 1, 0, 2, 1)
        self.buttonLayout.addWidget(self.pb_cam_right, 1, 4, 2, 1)
        self.buttonLayout.addWidget(self.scrollbar_cam_zoom, 1, 1, 1, 3)
        
    def _cam_slider_moved(self, value):
        self.cam_zpos = value;
        self.tb_cam_zpos.setText(QString(str(self.cam_zpos)))

    def _handle_pb_cam_up_clicked(self):
        if (self.cam_ypos <= 50):
            self.cam_ypos += 1;
        else:
            self.cam_ypos = 50;
        self.tb_cam_ypos.setText(QString(str(self.cam_ypos)))
    
    def _handle_pb_cam_down_clicked(self):
        if (self.cam_ypos >= -50):
            self.cam_ypos -= 1;
        else:
            self.cam_ypos = -50;
        self.tb_cam_ypos.setText(QString(str(self.cam_ypos)))
        
    def _handle_pb_cam_left_clicked(self):
        if (self.cam_xpos >= -50):
            self.cam_xpos -= 1;
        else:
            self.cam_xpos = -50;
        self.tb_cam_xpos.setText(QString(str(self.cam_xpos)))

    def _handle_pb_cam_right_clicked(self):
        if (self.cam_xpos <= 50):
            self.cam_xpos += 1;
        else:
            self.cam_xpos = 50;
        self.tb_cam_xpos.setText(QString(str(self.cam_xpos)))

if __name__ == '__main__':
    app=QApplication(sys.argv)
    camWidget = cameraWidget()
    camWidget.show()
    app.exec_()
