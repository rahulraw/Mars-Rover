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
        
        
        #self.rqt_ui.showFullScreen()
        # self.rqt_ui.resize(1200,600)
        self.rqt_ui.show()
        #context.add_widget(self.mainWin.main_window)
        
IdRole = QtCore.Qt.UserRole

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Main UI Setup
        #rp = rospkg.RosPack()
        #ui_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource','RoverUI_main.ui')
        #self.mainWindow = loadUi(ui_file)
        #self.main_window = QtGui.QMainWindow()
        #self.mainWindow = Ui_MainWindow()
        #self.mainWindow.setupUi(self.main_window)
        
        self.createActions()
        self.createMenus()
        self.createButtons()
        
        # IMU UI Dialog
        self.imuWidget = imu_dialog()
        
        # Main UI
        centralWidget = QtGui.QWidget()
        self.setCentralWidget(centralWidget)

		# 3D Orientation Widget
        self.glWidget = GLWidget()
        self.glWidget.setFixedSize(300,300)
        # self.glWidgetArea = QtGui.QScrollArea()
        # self.glWidgetArea.setWidget(self.glWidget)
        # self.glWidgetArea.setWidgetResizable(True)
        #self.glWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        #self.glWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        #self.glWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        # self.glWidgetArea.setMinimumSize(200, 200)
        
        # Orientation Widget
        self.orientWidget = OrientWidget()
        self.orientWidget.setFixedSize(300,300)

        # self.orientWidgetArea = QtGui.QScrollArea()
        # self.orientWidgetArea.setWidget(self.orientWidget)
        # self.orientWidgetArea.setWidgetResizable(True)
        # self.orientWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        # self.orientWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        # self.orientWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        # self.orientWidgetArea.setMinimumSize(200, 200)
        
        # Camera Widget
        self.camWidget = cameraWidget()
        self.camWidget.setFixedSize(500,600)
        
        # self.camWidgetArea = QtGui.QScrollArea()
        # self.camWidgetArea.setWidget(self.camWidget)
        # self.camWidgetArea.setWidgetResizable(True)
        # self.camWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        # self.camWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        # self.camWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        # self.camWidgetArea.setMinimumSize(600, 400)
        # self.camWidgetArea.setMaximumSize(600, 400)
        
        centralLayout = QtGui.QGridLayout()
        centralLayout.addWidget(self.glWidget, 0, 0, 1, 1)
        centralLayout.addWidget(self.orientWidget, 1, 0, 1, 1)
        centralLayout.addWidget(self.camWidget, 0, 1, 2, 1)
        # centralLayout.addWidget(self.pb_imu, 1, 1)
        # centralLayout.addWidget(self.pb_cam_start, 2, 1)        
        centralWidget.setLayout(centralLayout)

        self.setWindowTitle("Arthrobot Main Window")
        #self.resize(1200, 600)

    def about(self):
        QtGui.QMessageBox.about(self, "About Arthrobot Main Window",
                "The <b>Arthrobot</b> control window shows live camera stream and various sensor feedback information.")

    def createActions(self):
        self.openImuData = QtGui.QAction("Open IMU data window", self, shortcut="Ctrl+I", triggered=self._handle_pb_imu_clicked)
                
        self.exitAct = QtGui.QAction("E&xit", self, shortcut="Ctrl+Q", triggered=self.close)

        self.aboutAct = QtGui.QAction("&About", self, triggered=self.about)

        self.aboutQtAct = QtGui.QAction("About &Qt", self, triggered=QtGui.qApp.aboutQt)

    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu("&File")
        self.fileMenu.addAction(self.openImuData)
        self.fileMenu.addSeparator()
        self.fileMenu.addAction(self.exitAct)

        self.helpMenu = self.menuBar().addMenu("&Help")
        self.helpMenu.addAction(self.aboutAct)
        self.helpMenu.addAction(self.aboutQtAct)

    def createButtons(self):
        # IMU Data
        self.pb_imu = QtGui.QPushButton("IMU Data")
        self.pb_imu.clicked[bool].connect(self._handle_pb_imu_clicked)
        
        ## Camera Button
        self.pb_cam_start = QtGui.QPushButton("Start Camera")
        self.pb_cam_start.clicked[bool].connect(self._handle_pb_cam_start_clicked)       

    def _handle_pb_imu_clicked(self):
        self.imuWidget._dialog.show()
        rospy.Subscriber("/imu/data", Imu, self.imuWidget.updateImu)
        
    def _handle_pb_cam_start_clicked(self):
        self.camWidget.startCamera()
		
        
