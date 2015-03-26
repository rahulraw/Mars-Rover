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
from maps import gmapsWidget
from sensor_msgs.msg import Imu

class ArthrobotGui(Plugin):

    def __init__(self, context):
        super(ArthrobotGui, self).__init__(context)
        self.rqt_ui = MainWindow()
        self.rqt_ui.resize(1300,600)
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

        self.orientWidget = OrientWidget()
        self.orientWidget.setMinimumSize(300,300)
        self.orientWidget.setSizePolicy(
            QtGui.QSizePolicy.MinimumExpanding,
            QtGui.QSizePolicy.MinimumExpanding)
        
        self.camWidget = cameraWidget()
        self.camWidget.setMinimumSize(500,600)

        self.glWidget = GLWidget(self.orientWidget)
        self.glWidget.setMinimumSize(300,300)
        self.glWidget.setSizePolicy(
            QtGui.QSizePolicy.MinimumExpanding,
            QtGui.QSizePolicy.MinimumExpanding)

        self.mapWidget = gmapsWidget()
        self.mapWidget.setMinimumSize(100,100)
        self.mapWidget.setSizePolicy(
            QtGui.QSizePolicy.MinimumExpanding,
            QtGui.QSizePolicy.MinimumExpanding)

        self.controlArea = QtGui.QScrollArea()
        self.controlLayout = QtGui.QVBoxLayout()

        self.controlLayout.addWidget(self.pb_imu)
        self.controlLayout.addWidget(self.pb_cam_start)
        self.controlLayout.addWidget(self.pb_calibrateImu)
        # self.controlLayout.addWidget(self.pb_kill_imu)
        self.controlArea.setLayout(self.controlLayout)
        self.controlArea.setMinimumSize(100,100)
        self.controlArea.setSizePolicy(
            QtGui.QSizePolicy.MinimumExpanding,
            QtGui.QSizePolicy.MinimumExpanding)

        centralLayout = QtGui.QGridLayout()
        centralLayout.addWidget(self.glWidget, 0, 0, 2, 1)
        centralLayout.addWidget(self.orientWidget, 2, 0, 2, 1)
        centralLayout.addWidget(self.camWidget, 0, 1, 4, 2)
        centralLayout.addWidget(self.mapWidget, 0, 3, 3, 1)
        # centralLayout.addWidget(self.controlArea, 3, 3, 1, 1)

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

        self.pb_kill_imu = QtGui.QPushButton('Kill IMU')
        self.pb_kill_imu.clicked[bool].connect(self._handle_pb_killImu_clicked)

    def _handle_pb_imu_clicked(self):
        self.imuWidget.startImu()
        self.imuWidget._dialog.show()

    def _handle_pb_cam_start_clicked(self):
        self.camWidget.startCamera()

    def _handle_pb_calibrateImu_clicked(self):
        self.glWidget.calibrateImu()

    def _handle_pb_killImu_clicked(self):
        self.imuWidget.exitImu()
