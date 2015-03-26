import os, sys, random
import rospy, rospkg, rosbag
import pyqtgraph as pg
import numpy as np

from qt_gui.plugin import Plugin

from PyQt4 import QtCore, QtGui
from PyQt4.uic import loadUi
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from imudata import imuWidget
from rover3d import GLWidget
from rover2d import OrientWidget
from camerastream import cameraWidget
from maps import gmapsWidget

from steering.msg import RoverInfo

try:
    from PyQt4.QtCore import QString
except ImportError:
    QString = str

class ArthrobotGui(Plugin):

    def __init__(self, context):
        super(ArthrobotGui, self).__init__(context)
        self.rqt_ui = MainWindow()
        self.rqt_ui.resize(1300,600)
        self.rqt_ui.show()

class MainWindow(QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        rospy.Subscriber('RoverInfo', RoverInfo, self.updateSensorValues)

        self.createActions()
        self.createMenus()
        self.createButtons()
        self.createLabels()
        self.createPlotter()
        
        centralWidget = QtGui.QWidget()
        self.setCentralWidget(centralWidget)

        self.imuWidget = imuWidget()

        self.orientWidget = OrientWidget()
        self.orientWidget.setMinimumSize(300,300)
        self.orientWidget.setSizePolicy(
            QtGui.QSizePolicy.MinimumExpanding,
            QtGui.QSizePolicy.MinimumExpanding)
        
        self.camWidget = cameraWidget()
        self.camWidget.setMinimumSize(500,600)
        self.camWidget.setFocus()

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

        self.controlTab = QtGui.QTabWidget()
        self.controlTab.setMinimumSize(100,100)
        self.controlTab.setSizePolicy(
            QtGui.QSizePolicy.MinimumExpanding,
            QtGui.QSizePolicy.MinimumExpanding)

        self.controlArea = QtGui.QScrollArea()
        self.controlLayout = QtGui.QVBoxLayout()

        # self.controlLayout.addWidget(self.pb_imu)
        self.controlLayout.addWidget(self.pb_cam_start)
        self.controlLayout.addWidget(self.pb_calibrateImu)
        # self.controlLayout.addWidget(self.pb_kill_imu)
        self.controlArea.setLayout(self.controlLayout)

        self.sensingArea = QtGui.QScrollArea()
        self.sensingLayout = QtGui.QGridLayout()
        self.createSensorWidget()

        self.sensingArea.setLayout(self.sensingLayout)

        self.controlTab.addTab(self.controlArea, "Controls")
        self.controlTab.addTab(self.sensingArea, "Sensor Data")
        self.controlTab.addTab(self.imuWidget._widget, "IMU Data")

        centralLayout = QtGui.QGridLayout()
        centralLayout.addWidget(self.glWidget, 0, 0, 2, 1)
        centralLayout.addWidget(self.orientWidget, 2, 0, 2, 1)
        centralLayout.addWidget(self.camWidget, 0, 1, 4, 2)
        centralLayout.addWidget(self.mapWidget, 0, 3, 3, 1)
        centralLayout.addWidget(self.controlTab, 3, 3, 1, 1)

        centralWidget.setLayout(centralLayout)

        self.setWindowTitle('Arthrobot Main Window')

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.updatePlot)
        self.timer.start(1000)

    def about(self):
        QtGui.QMessageBox.about(self, 'About Arthrobot Main Window', 
            'The <b>Arthrobot</b> control window shows live camera stream and various sensor feedback information.')

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

    def createLabels(self):
        self.BLCurrent = 0.0
        self.BRCurrent = 0.0
        self.FLCurrent = 0.0
        self.FRCurrent = 0.0

        self.BLVoltage = 0.0
        self.BRVoltage = 0.0
        self.FLVoltage = 0.0
        self.FRVoltage = 0.0

        self.label_roboclaw = QtGui.QLabel('Wheel Motor Controller Readings')

        self.radiobutton_current = QtGui.QRadioButton('Current (mA)')
        self.radiobutton_current.setChecked(True)
        self.label_current1 = QtGui.QLabel('BL Current')
        self.label_current2 = QtGui.QLabel('BR Current')
        self.label_current3 = QtGui.QLabel('FL Current')
        self.label_current4 = QtGui.QLabel('FR Current')
        self.label_current_value1 = QtGui.QLabel(QString(self.BLCurrent))
        self.label_current_value2 = QtGui.QLabel(QString(self.BRCurrent))
        self.label_current_value3 = QtGui.QLabel(QString(self.FLCurrent))
        self.label_current_value4 = QtGui.QLabel(QString(self.FRCurrent))
        self.label_current_value1.setStyleSheet('color: red')
        self.label_current_value2.setStyleSheet('color: yellow')
        self.label_current_value3.setStyleSheet('color: green')
        self.label_current_value4.setStyleSheet('color: blue')

        self.radiobutton_voltage = QtGui.QRadioButton('Voltage (V)')
        self.label_voltage1 = QtGui.QLabel('BL Voltage')
        self.label_voltage2 = QtGui.QLabel('BR Voltage')
        self.label_voltage3 = QtGui.QLabel('FL Voltage')
        self.label_voltage4 = QtGui.QLabel('FR Voltage')
        self.label_voltage_value1 = QtGui.QLabel(QString(self.BLVoltage))
        self.label_voltage_value2 = QtGui.QLabel(QString(self.BRVoltage))
        self.label_voltage_value3 = QtGui.QLabel(QString(self.FLVoltage))
        self.label_voltage_value4 = QtGui.QLabel(QString(self.FRVoltage))
        self.label_voltage_value1.setStyleSheet('color: red')
        self.label_voltage_value2.setStyleSheet('color: yellow')
        self.label_voltage_value3.setStyleSheet('color: green')
        self.label_voltage_value4.setStyleSheet('color: blue')

    def createSensorWidget(self):
        self.sensingLayout.addWidget(self.label_roboclaw, 0, 0, 1, 5)
        self.sensingLayout.addWidget(self.sensorPlot, 1, 2, 10, 3)

        self.sensingLayout.addWidget(self.radiobutton_current, 1, 0, 1, 2)
        self.sensingLayout.addWidget(self.label_current1, 2, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_current2, 3, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_current3, 4, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_current4, 5, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_current_value1, 2, 1, 1, 1)
        self.sensingLayout.addWidget(self.label_current_value2, 3, 1, 1, 1)
        self.sensingLayout.addWidget(self.label_current_value3, 4, 1, 1, 1)
        self.sensingLayout.addWidget(self.label_current_value4, 5, 1, 1, 1)

        self.sensingLayout.addWidget(self.radiobutton_voltage, 6, 0, 1, 2)
        self.sensingLayout.addWidget(self.label_voltage1, 7, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_voltage2, 8, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_voltage3, 9, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_voltage4, 10, 0, 1, 1)
        self.sensingLayout.addWidget(self.label_voltage_value1, 7, 1, 1, 1)
        self.sensingLayout.addWidget(self.label_voltage_value2, 8, 1, 1, 1)
        self.sensingLayout.addWidget(self.label_voltage_value3, 9, 1, 1, 1)
        self.sensingLayout.addWidget(self.label_voltage_value4, 10, 1, 1, 1)

        # self.label_cam_zoom.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        # self.label_cam_rotate.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)
        # self.label_cam_pitch.setAlignment(Qt.AlignHCenter | Qt.AlignBottom)

    def createPlotter(self):
        self.sensorPlot = pg.PlotWidget(title = "Real Time Sensor Readings")
        self.sensorPlot.setLabel('bottom', 'Time', units='s')
        # self.sensorPlot.setXRange(-10,0)

        self.startTime = pg.ptime.time()
        self.chunkSize = 100
        self.maxChunks = 10

        self.BL_curve = self.sensorPlot.plot(pen=QPen(QColor(255,0,0)))
        self.BR_curve = self.sensorPlot.plot(pen=QPen(QColor(255,255,0)))
        self.FL_curve = self.sensorPlot.plot(pen=QPen(QColor(0,255,0)))
        self.FR_curve = self.sensorPlot.plot(pen=QPen(QColor(0,0,255)))

        self.BLCurrent_sensorData = [] #np.random.normal(size=10)
        self.BRCurrent_sensorData = [] #np.random.normal(size=10)
        self.FLCurrent_sensorData = [] #np.random.normal(size=10)
        self.FRCurrent_sensorData = [] #np.random.normal(size=10)

        self.BLVoltage_sensorData = [] #np.random.normal(size=10)
        self.BRVoltage_sensorData = [] #np.random.normal(size=10)
        self.FLVoltage_sensorData = [] #np.random.normal(size=10)
        self.FRVoltage_sensorData = [] #np.random.normal(size=10)

        self.sensorPtr = 0

    def updatePlot(self):
        self.sensorPtr += 1

        if (self.radiobutton_current.isChecked()):
            self.sensorPlot.setLabel('left', 'Current', units='mA')

            self.BL_curve.setData(self.BLCurrent_sensorData)
            self.BL_curve.setPos(self.sensorPtr, 0)

            self.BR_curve.setData(self.BRCurrent_sensorData)
            self.BR_curve.setPos(self.sensorPtr, 0)

            self.FL_curve.setData(self.FLCurrent_sensorData)
            self.FL_curve.setPos(self.sensorPtr, 0)

            self.FR_curve.setData(self.FRCurrent_sensorData)
            self.FR_curve.setPos(self.sensorPtr, 0)
        else:
            self.sensorPlot.setLabel('left', 'Voltage', units='V')

            self.BL_curve.setData(self.BLVoltage_sensorData)
            self.BL_curve.setPos(self.sensorPtr, 0)

            self.BR_curve.setData(self.BRVoltage_sensorData)
            self.BR_curve.setPos(self.sensorPtr, 0)

            self.FL_curve.setData(self.FLVoltage_sensorData)
            self.FL_curve.setPos(self.sensorPtr, 0)

            self.FR_curve.setData(self.FRVoltage_sensorData)
            self.FR_curve.setPos(self.sensorPtr, 0)

    ############################
    # MAIN FORM EVENT HANDLERS # 
    ############################

    def updateSensorValues(self, value):
        self.BLCurrent = float(value.back_left_current)
        self.BRCurrent = float(value.back_right_current)
        self.FLCurrent = float(value.front_left_current)
        self.FRCurrent = float(value.front_right_current)

        self.BLVoltage = float(value.back_left_voltage)
        self.BRVoltage = float(value.back_right_voltage)
        self.FLVoltage = float(value.front_left_voltage)
        self.FRVoltage = float(value.front_right_voltage)

        self.label_current_value1 = QString(self.BLCurrent)
        self.label_current_value2 = QString(self.BRCurrent)
        self.label_current_value3 = QString(self.FLCurrent)
        self.label_current_value4 = QString(self.FRCurrent)

        self.label_voltage_value1 = QString(self.BLVoltage)
        self.label_voltage_value2 = QString(self.BRVoltage)
        self.label_voltage_value3 = QString(self.FLVoltage)
        self.label_voltage_value4 = QString(self.FRVoltage)

        self.BLCurrent_sensorData[:-1] = self.BLCurrent_sensorData[1:]
        self.BLCurrent_sensorData[-1] = self.BLCurrent

        self.BRCurrent_sensorData[:-1] = self.BRCurrent_sensorData[1:]
        self.BRCurrent_sensorData[-1] = self.BRCurrent

        self.FLCurrent_sensorData[:-1] = self.FLCurrent_sensorData[1:]
        self.FLCurrent_sensorData[-1] = self.FLCurrent

        self.FRCurrent_sensorData[:-1] = self.FRCurrent_sensorData[1:]
        self.FRCurrent_sensorData[-1] = self.FRCurrent

        self.BLVoltage_sensorData[:-1] = self.BLVoltage_sensorData[1:]
        self.BLVoltage_sensorData[-1] = self.BLVoltage

        self.BRVoltage_sensorData[:-1] = self.BRVoltage_sensorData[1:]
        self.BRVoltage_sensorData[-1] = self.BRVoltage

        self.FLVoltage_sensorData[:-1] = self.FLVoltage_sensorData[1:]
        self.FLVoltage_sensorData[-1] = self.FLVoltage

        self.FRVoltage_sensorData[:-1] = self.FRVoltage_sensorData[1:]
        self.FRVoltage_sensorData[-1] = self.FRVoltage

        self.update()

    def _handle_pb_imu_clicked(self):
        self.imuWidget.startImu()
        # self.imuWidget._widget.show()

    def _handle_pb_cam_start_clicked(self):
        self.camWidget.startCamera()

    def _handle_pb_calibrateImu_clicked(self):
        self.glWidget.calibrateImu()

    def _handle_pb_killImu_clicked(self):
        self.imuWidget.exitImu()
