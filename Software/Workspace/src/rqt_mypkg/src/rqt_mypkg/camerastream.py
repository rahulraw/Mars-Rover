#Embedded file name: /home/jerbotron/Documents/uwrobotics.uwmrt/Software/Workspace/src/rqt_mypkg/src/rqt_mypkg/camerastream.py
import numpy as np
import rospy
import serial
import sys
import cv, cv2

from lib.bridges.bwbridge import BWBridge
from lib.bridges.rgbbridge import RGBBridge
from lib.messages import Messages
from lib.webcam import Webcam
from config import Config
from rqt_mypkg.msg import CameraMountInfo
from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from camera import Camera

try:
    from PyQt4.QtCore import QString
except ImportError:
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
        print 'cam thread run'
        rospy.Subscriber(self.topic, self.bridge.getType(), self.callback)

    def callback(self, video):
        self.cv_image = None
        self.cv_image = self.bridge.from_imgmsg(video)

        if self.cv_image != None:
            height, width, bytesPerComponent = self.cv_image.shape
            bytesPerLine = 3 * width
            cv2.cvtColor(self.cv_image, cv.CV_BGR2RGB, self.cv_image)
            self.image = QImage(self.cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)

        self.emit(QtCore.SIGNAL('start(QImage)'), self.image)

class cameraWidget(QtGui.QWidget):

    def __init__(self, parent = None):
        super(cameraWidget, self).__init__(parent)
        # self.camera = Camera()
        self.webcam = Webcam(Config.scale)
        self.topic = rospy.get_param('~topic', 'video_stream')
        self.node = rospy.get_param('~node', 'receiver')
        if Config.gray:
            self.bridge = BWBridge()
        else:
            self.bridge = RGBBridge()

        self.cam_xpos = 0
        self.cam_ypos = 0
        self.cam_zpos = 0
        
        self.controlLayout = QtGui.QGridLayout()
        self.camControlArea = QtGui.QScrollArea()
        self.camControlArea.setLayout(self.controlLayout)
        self.camControlArea.setFixedSize(500, 200)
        self.createLabels()
        self.createTextBoxes()
        self.createScrollbars()
        self.createControls()
        self.cameraWindow = QtGui.QLabel(self)
        self.cameraWindow.setScaledContents(True)
        self.cameraWindow.setFixedSize(500, 400)
        self.cameraBox = QtGui.QVBoxLayout()
        self.cameraBox.addWidget(self.cameraWindow)
        self.cameraBox.addWidget(self.camControlArea)
        self.setLayout(self.cameraBox)
        self.setFixedSize(500, 600)

        self.topic = 'CameraPositions'
        self.node = 'CameraMount'
        self.info_pub = rospy.Publisher('CameraMountInfo', CameraMountInfo, queue_size=10)
        self.rate = rospy.Rate(5)

    def publishInfo(self):
        cam_info = CameraMountInfo()
        cam_info.x_pos = self.cam_xpos
        cam_info.y_pos = self.cam_ypos
        cam_info.zoom = self.cam_zpos
        self.info_pub.publish(cam_info)

    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_J:
            self._handle_pb_cam_left_event()
        elif e.key() == QtCore.Qt.Key_L:
            self._handle_pb_cam_right_event()
        elif e.key() == QtCore.Qt.Key_I:
            self._handle_pb_cam_up_event()
        elif e.key() == QtCore.Qt.Key_K:
            self._handle_pb_cam_down_event()
        elif e.key() == QtCore.Qt.Key_Plus:
            self._handle_pb_zoom_in_event()
        elif e.key() == QtCore.Qt.Key_Minus:
            self._handle_pb_zoom_out_event()
        self.publishInfo()

    def startCamera(self):
        # self.camera.start()
        self.cameraThread = camThread()
        self.connect(self.cameraThread, QtCore.SIGNAL('start(QImage)'), self.start, Qt.QueuedConnection)
        self.cameraThread.start()

    def setImage(self, pixmap):
        self.cameraWindow.setPixmap(pixmap)

    def start(self, cv_image):
        pixmap = QPixmap.fromImage(cv_image)
        self.setImage(pixmap)

    def createLabels(self):
        self.label_cam_zoom_in = QtGui.QLabel('+')
        self.label_cam_zoom_out = QtGui.QLabel('-')
        self.label_cam_zoom = QtGui.QLabel('zoom')
        self.label_cam_xpos = QtGui.QLabel('x pos')
        self.label_cam_ypos = QtGui.QLabel('y pos')
        self.label_cam_zoom_in.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.label_cam_zoom_out.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.label_cam_zoom.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        self.label_cam_xpos.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        self.label_cam_ypos.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)

    def createTextBoxes(self):
        self.tb_cam_xpos = QtGui.QLineEdit()
        self.tb_cam_ypos = QtGui.QLineEdit()
        self.tb_cam_zpos = QtGui.QLineEdit()
        self.tb_cam_xpos.setReadOnly(True)
        self.tb_cam_ypos.setReadOnly(True)
        self.tb_cam_zpos.setReadOnly(True)
        self.tb_cam_xpos.setText(QString('0'))
        self.tb_cam_ypos.setText(QString('0'))
        self.tb_cam_zpos.setText(QString('0'))

    def createScrollbars(self):
        self.scrollbar_x_pos = QtGui.QScrollBar()
        self.scrollbar_x_pos.setOrientation(QtCore.Qt.Horizontal)
        self.scrollbar_x_pos.setMinimum(-50)
        self.scrollbar_x_pos.setMaximum(50)
        self.scrollbar_x_pos.connect(self.scrollbar_x_pos, QtCore.SIGNAL('sliderMoved(int)'), self._cam_slider_x_moved)
        self.scrollbar_y_pos = QtGui.QScrollBar()
        self.scrollbar_y_pos.setOrientation(QtCore.Qt.Vertical)
        self.scrollbar_y_pos.setMinimum(-50)
        self.scrollbar_y_pos.setMaximum(50)
        self.scrollbar_y_pos.connect(self.scrollbar_y_pos, QtCore.SIGNAL('sliderMoved(int)'), self._cam_slider_y_moved)
        self.scrollbar_cam_zoom = QtGui.QScrollBar()
        self.scrollbar_cam_zoom.setOrientation(QtCore.Qt.Horizontal)
        self.scrollbar_cam_zoom.setMinimum(0)
        self.scrollbar_cam_zoom.setMaximum(100)
        self.scrollbar_cam_zoom.connect(self.scrollbar_cam_zoom, QtCore.SIGNAL('sliderMoved(int)'), self._cam_slider_zoom_moved)

    def createButtons(self):
        pass

    def createControls(self):
        self.controlLayout.addWidget(self.scrollbar_x_pos, 0, 0, 1, 4)
        self.controlLayout.addWidget(self.scrollbar_y_pos, 0, 4, 5, 1)
        self.controlLayout.addWidget(self.scrollbar_cam_zoom, 2, 0, 1, 4)
        self.controlLayout.addWidget(self.label_cam_xpos, 3, 0, 1, 1)
        self.controlLayout.addWidget(self.label_cam_ypos, 3, 3, 1, 1)
        self.controlLayout.addWidget(self.label_cam_zoom, 3, 1, 1, 2)
        self.controlLayout.addWidget(self.tb_cam_xpos, 4, 0, 1, 1)
        self.controlLayout.addWidget(self.tb_cam_ypos, 4, 3, 1, 1)
        self.controlLayout.addWidget(self.tb_cam_zpos, 4, 1, 1, 2)

    def _cam_slider_x_moved(self, value):
        self.cam_xpos = value
        self.tb_cam_xpos.setText(QString(self.cam_xpos))

    def _cam_slider_y_moved(self, value):
        self.cam_ypos = -value
        self.tb_cam_ypos.setText(QString(self.cam_ypos))

    def _cam_slider_zoom_moved(self, value):
        self.cam_zpos = value
        self.tb_cam_zpos.setText(QString(self.cam_zpos))

    def _handle_pb_cam_left_event(self):
        if self.cam_xpos > -50:
            self.cam_xpos -= 1
        else:
            self.cam_xpos = -50
        self.tb_cam_xpos.setText(QString(self.cam_xpos))
        self.scrollbar_x_pos.setValue(self.cam_xpos)

    def _handle_pb_cam_right_event(self):
        if self.cam_xpos < 50:
            self.cam_xpos += 1
        else:
            self.cam_xpos = 50
        self.tb_cam_xpos.setText(QString(self.cam_xpos))
        self.scrollbar_x_pos.setValue(self.cam_xpos)

    def _handle_pb_cam_up_event(self):
        if self.cam_ypos < 50:
            self.cam_ypos += 1
        else:
            self.cam_ypos = 50
        self.tb_cam_ypos.setText(QString(self.cam_ypos))
        self.scrollbar_y_pos.setValue(-self.cam_ypos)

    def _handle_pb_cam_down_event(self):
        if self.cam_ypos > -50:
            self.cam_ypos -= 1
        else:
            self.cam_ypos = -50
        self.tb_cam_ypos.setText(QString(self.cam_ypos))
        self.scrollbar_y_pos.setValue(-self.cam_ypos)

    def _handle_pb_zoom_in_event(self):
        if self.cam_zpos < 100:
            self.cam_zpos += 1
        else:
            self.cam_zpos = 100
        self.tb_cam_zpos.setText(QString(self.cam_zpos))
        self.scrollbar_cam_zoom.setValue(self.cam_zpos)

    def _handle_pb_zoom_out_event(self):
        if self.cam_zpos > 0:
            self.cam_zpos -= 1
        else:
            self.cam_zpos = 0
        self.tb_cam_zpos.setText(QString(self.cam_zpos))
        self.scrollbar_cam_zoom.setValue(self.cam_zpos)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    camWidget = cameraWidget()
    camWidget.show()
    app.exec_()
