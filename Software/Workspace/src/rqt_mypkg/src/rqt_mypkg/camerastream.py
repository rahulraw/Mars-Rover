import numpy as np
import rospy, rospkg
import serial
import sys, os
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
        self.isPublished = False

        if Config.gray:
            self.bridge = BWBridge()
        else:
            self.bridge = RGBBridge()

        self.cameraOn = False

        self.cam_ymin = 140
        self.cam_ymax = 170

        self.cam_xpos = 1
        self.cam_ypos = (self.cam_ymin + self.cam_ymax)/2
        self.cam_zpos = 1
        
        self.controlLayout = QtGui.QGridLayout()
        self.camControlArea = QtGui.QScrollArea()
        self.camControlArea.setLayout(self.controlLayout)
        self.camControlArea.setMinimumSize(500, 125)
        self.camControlArea.setFixedHeight(125)

        self.createLabels()
        self.createScrollbars()
        self.createDials()
        self.createTextBoxes()
        self.createControls()
        self.cameraWindow = QtGui.QLabel(self)
        self.cameraWindow.setScaledContents(True)
        self.cameraWindow.setMinimumSize(500, 500)
        self.cameraBox = QtGui.QVBoxLayout()
        self.cameraBox.addWidget(self.cameraWindow)
        self.cameraBox.addWidget(self.camControlArea)
        self.setLayout(self.cameraBox)
        # self.setFixedSize(500, 600)

        rp = rospkg.RosPack()
        self.imgPath = os.path.join(rp.get_path('rqt_mypkg'), 'src/rqt_mypkg/resource', 'arthrobot.jpg')
        self.cameraWindow.setPixmap(QtGui.QPixmap(self.imgPath))

        self.topic = 'CameraPositions'
        self.node = 'CameraMount'
        self.info_pub = rospy.Publisher('CameraMountInfo', CameraMountInfo, queue_size=10)
        self.rate = rospy.Rate(5)

    def publishInfo(self):
        cam_info = CameraMountInfo()
        cam_info.x_pos = self.cam_xpos
        cam_info.y_pos = self.cam_ymin + (self.cam_ymax - self.cam_ypos)
        cam_info.zoom = self.cam_zpos
        self.info_pub.publish(cam_info)

    def keyPressEvent(self, e):
        # print("key pressed")
        if e.key() == QtCore.Qt.Key_J:
            self._handle_pb_cam_left_event()
        elif e.key() == QtCore.Qt.Key_L:
            self._handle_pb_cam_right_event()
        elif e.key() == QtCore.Qt.Key_I:
            self._handle_pb_cam_up_event()
        elif e.key() == QtCore.Qt.Key_K:
            self._handle_pb_cam_down_event()
        elif e.key() == QtCore.Qt.Key_Z:
            self._handle_pb_zoom_in_event()
        elif e.key() == QtCore.Qt.Key_X:
            self._handle_pb_zoom_out_event()
        if (not self.isPublished):
            self.publishInfo()
            self.isPublished = True

    def keyReleaseEvent(self, e):
        if e.key() == QtCore.Qt.Key_J or e.key() == QtCore.Qt.Key_L:
            self.cam_xpos = 1
            self.displayxposText()
            self.dial_rotate.setValue(self.cam_xpos)
        elif e.key() == QtCore.Qt.Key_Z or e.key() == QtCore.Qt.Key_X:
            self.cam_zpos = 1
            self.displayzoomText()
            self.dial_zoom.setValue(self.cam_zpos)
        if (self.isPublished):
            self.publishInfo()
            self.isPublished = False

    def startCamera(self):
        # self.camera.start()
        self.cameraOn = True
        self.cameraThread = camThread()
        self.connect(self.cameraThread, QtCore.SIGNAL('start(QImage)'), self.start, Qt.QueuedConnection)
        self.cameraThread.start()

    def stopCamera(self):
        self.cameraOn = False     

    def setImage(self, pixmap):
        self.cameraWindow.setPixmap(pixmap)

    def start(self, cv_image):
        if (self.cameraOn == False):
            self.cameraThread.exit()
            self.cameraWindow.setPixmap(QtGui.QPixmap(self.imgPath))
        else:
            pixmap = QPixmap.fromImage(cv_image)
            self.setImage(pixmap)

    def createLabels(self):
        self.label_cam_zoom = QtGui.QLabel('Zoom')
        self.label_cam_rotate = QtGui.QLabel('Rotate')
        self.label_cam_pitch = QtGui.QLabel('Pitch')
        self.label_cam_zoom.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        self.label_cam_rotate.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        self.label_cam_pitch.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)

    def createTextBoxes(self):
        self.tb_cam_rotate = QtGui.QLineEdit()
        self.tb_cam_pitch = QtGui.QLineEdit()
        self.tb_cam_zoom = QtGui.QLineEdit()
        self.tb_cam_rotate.setReadOnly(True)
        self.tb_cam_pitch.setReadOnly(True)
        self.tb_cam_zoom.setReadOnly(True)
        self.displayxposText()
        self.tb_cam_pitch.setText(QString(self.cam_ypos))
        self.displayzoomText()

        self.tb_ymin = QtGui.QLineEdit()
        self.tb_ymax = QtGui.QLineEdit()
        self.tb_ymin.connect(self.tb_ymin, QtCore.SIGNAL('textChanged(QString)'), self._tb_ymin_changed)
        self.tb_ymax.connect(self.tb_ymax, QtCore.SIGNAL('textChanged(QString)'), self._tb_ymax_changed)
        self.tb_ymin.setText(QString(self.cam_ymin))
        self.tb_ymax.setText(QString(self.cam_ymax))

    def createScrollbars(self):
        self.scrollbar_pitch = QtGui.QScrollBar()
        self.scrollbar_pitch.setOrientation(QtCore.Qt.Horizontal)
        self.scrollbar_pitch.setMinimum(self.cam_ymin)
        self.scrollbar_pitch.setMaximum(self.cam_ymax)
        self.scrollbar_pitch.setDisabled(True)

        self.scrollbar_pitch.setValue(self.cam_ypos)

    def createDials(self):
        self.dial_rotate = QtGui.QDial()
        self.dial_rotate.setRange(0,2)
        self.dial_rotate.setDisabled(True)
        self.dial_rotate.setNotchesVisible(True)

        self.dial_zoom = QtGui.QDial()
        self.dial_zoom.setRange(0,2)
        self.dial_zoom.setDisabled(True)
        self.dial_zoom.setNotchesVisible(True)

        self.dial_rotate.setValue(self.cam_xpos)
        self.dial_zoom.setValue(self.cam_zpos)

    def createButtons(self):
        pass

    def createControls(self):
        self.controlLayout.addWidget(self.label_cam_rotate, 0, 0, 1, 1)
        self.controlLayout.addWidget(self.label_cam_zoom, 0, 1, 1, 1)
        self.controlLayout.addWidget(self.label_cam_pitch, 0, 2, 1, 4)

        self.controlLayout.addWidget(self.dial_rotate, 1, 0, 2, 1)
        self.controlLayout.addWidget(self.dial_zoom, 1, 1, 2, 1)
        self.controlLayout.addWidget(self.scrollbar_pitch, 1, 2, 2, 4)

        self.controlLayout.addWidget(self.tb_cam_rotate, 3, 0, 1, 1)
        self.controlLayout.addWidget(self.tb_cam_zoom, 3, 1, 1, 1)
        self.controlLayout.addWidget(self.tb_cam_pitch, 3, 2, 1, 4)

        # self.controlLayout.addWidget(self.tb_ymin, 4, 0, 1, 2)
        # self.controlLayout.addWidget(self.tb_ymax, 4, 2, 1, 2)

    def _cam_slider_x_moved(self, value):
        self.cam_xpos = value
        self.displayxposText()
        self.publishInfo()

    def _cam_slider_y_moved(self, value):
        self.cam_ypos = self.cam_ymin + (self.cam_ymax-value)
        self.tb_cam_pitch.setText(QString(self.cam_ypos))
        self.publishInfo()

    def _cam_slider_zoom_moved(self, value):
        self.cam_zpos = value
        self.displayzoomText()
        self.publishInfo()

    def _handle_pb_cam_left_event(self):
        self.cam_xpos = 0
        self.dial_rotate.setValue(self.cam_xpos)
        self.displayxposText()

    def _handle_pb_cam_right_event(self):
        self.cam_xpos = 2
        self.dial_rotate.setValue(self.cam_xpos)
        self.displayxposText()

    def displayxposText(self):
        if (self.cam_xpos == 0):
            self.tb_cam_rotate.setText("Rotate Left")
        elif (self.cam_xpos == 1):
            self.tb_cam_rotate.setText("Stopped")
        elif (self.cam_xpos == 2):
            self.tb_cam_rotate.setText("Rotate Right")

    def displayzoomText(self):
        if (self.cam_zpos == 0):
            self.tb_cam_zoom.setText("Zooming Out")
        elif (self.cam_zpos == 1):
            self.tb_cam_zoom.setText("Stopped")
        elif (self.cam_zpos == 2):
            self.tb_cam_zoom.setText("Zooming In")

    def _handle_pb_cam_up_event(self):
        if (self.cam_ypos < self.cam_ymax):
            self.cam_ypos += 1
        else:
            self.cam_ypos = self.cam_ymax
        self.tb_cam_pitch.setText(QString(self.cam_ypos))
        # self.scrollbar_pitch.setValue(self.cam_ymin + (self.cam_ymax-self.cam_ypos))
        self.scrollbar_pitch.setValue(self.cam_ypos)

    def _handle_pb_cam_down_event(self):
        if (self.cam_ypos > self.cam_ymin):
            self.cam_ypos -= 1
        else:
            self.cam_ypos = self.cam_ymin
        self.tb_cam_pitch.setText(QString(self.cam_ypos))
        # self.scrollbar_pitch.setValue(self.cam_ymin + (self.cam_ymax-self.cam_ypos))
        self.scrollbar_pitch.setValue(self.cam_ypos)

    def _handle_pb_zoom_in_event(self):
        self.cam_zpos = 0
        self.dial_zoom.setValue(self.cam_zpos)
        self.displayzoomText()

    def _handle_pb_zoom_out_event(self):
        self.cam_zpos = 2
        self.dial_zoom.setValue(self.cam_zpos)
        self.displayzoomText()

    def _tb_ymin_changed(self, value):
        self.cam_ymin = int(value)
        self.scrollbar_pitch.setMinimum(self.cam_ymin)
        self.scrollbar_pitch.setMaximum(self.cam_ymax)
        self.update()

    def _tb_ymax_changed(self, value):
        self.cam_ymax = int(value)
        self.scrollbar_pitch.setMaximum(self.cam_ymax)
        self.scrollbar_pitch.setMinimum(self.cam_ymin)
        self.update()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    camWidget = cameraWidget()
    camWidget.show()
    app.exec_()
