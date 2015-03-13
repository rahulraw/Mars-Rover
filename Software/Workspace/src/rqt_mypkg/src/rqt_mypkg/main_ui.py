#Embedded file name: /home/jerbotron/Documents/uwrobotics.uwmrt/Software/Workspace/src/rqt_mypkg/src/rqt_mypkg/main_ui.py
import os, sys, random
import rospy, rospkg, rosbag
from qt_gui.plugin import Plugin
from PyQt4 import QtCore, QtGui
from PyQt4.uic import loadUi
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from imudata import imu_dialog
from rover3d import GLWidget
from rover2d import OrientWidget
from camerastream import cameraWidget
from sensor_msgs.msg import Imu

class ArthrobotGui(Plugin):

    def __init__(self, context):
        super(ArthrobotGui, self).__init__(context)
        self.rqt_ui = MainWindow()
        self.rqt_ui.show()


class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()

        self.createActions()
        self.createMenus()
        self.createButtons()
        
        centralWidget = QtGui.QWidget()
        self.setCentralWidget(centralWidget)

        self.imuWidget = imu_dialog()
        self.glWidget = GLWidget()
        self.orientWidget = OrientWidget()
        self.camWidget = cameraWidget()

        self.controlArea = QtGui.QScrollArea()
        self.controlLayout = QtGui.QVBoxLayout()

        self.controlLayout.addWidget(self.pb_imu)
        self.controlLayout.addWidget(self.pb_cam_start)
        self.controlLayout.addWidget(self.pb_calibrateImu)
        self.controlArea.setLayout(self.controlLayout)
        self.controlArea.setFixedSize(200,600)

        centralLayout = QtGui.QGridLayout()
        centralLayout.addWidget(self.glWidget, 0, 0, 1, 1)
        centralLayout.addWidget(self.orientWidget, 1, 0, 1, 1)
        centralLayout.addWidget(self.camWidget, 0, 1, 2, 1)
        centralLayout.addWidget(self.controlArea, 0, 2, 3, 2)

        centralWidget.setLayout(centralLayout)

        self.setWindowTitle('Arthrobot Main Window')

    def about(self):
        QtGui.QMessageBox.about(self, 'About Arthrobot Main Window', 'The <b>Arthrobot</b> control window shows live camera stream and various sensor feedback information.')

    def createActions(self):
        self.openImuData = QtGui.QAction('Open IMU data window', self, shortcut='Ctrl+I', triggered=self._handle_pb_imu_clicked)
        self.exitAct = QtGui.QAction('E&xit', self, shortcut='Ctrl+Q', triggered=self.close)
        self.aboutAct = QtGui.QAction('&About', self, triggered=self.about)
        self.aboutQtAct = QtGui.QAction('About &Qt', self, triggered=QtGui.qApp.aboutQt)

    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu('&File')
        self.fileMenu.addAction(self.openImuData)
        self.fileMenu.addSeparator()
        self.fileMenu.addAction(self.exitAct)
        self.helpMenu = self.menuBar().addMenu('&Help')
        self.helpMenu.addAction(self.aboutAct)
        self.helpMenu.addAction(self.aboutQtAct)

    def createButtons(self):
        self.pb_imu = QtGui.QPushButton('IMU Data')
        self.pb_imu.clicked[bool].connect(self._handle_pb_imu_clicked)
        self.pb_cam_start = QtGui.QPushButton('Start Camera')
        self.pb_cam_start.clicked[bool].connect(self._handle_pb_cam_start_clicked)
        self.pb_calibrateImu = QtGui.QPushButton('Calibrate IMU')
        self.pb_calibrateImu.clicked[bool].connect(self._handle_pb_calibrateImu_clicked)

    def _handle_pb_imu_clicked(self):
        self.imuWidget._dialog.show()

    def _handle_pb_cam_start_clicked(self):
        self.camWidget.startCamera()

    def _handle_pb_calibrateImu_clicked(self):
        self.glWidget.calibrateImu()
