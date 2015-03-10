import os, sys, random
import rospy, rospkg, rosbag
import cv

from qt_gui.plugin import Plugin

from PyQt4 import QtCore, QtGui
from PyQt4.uic import loadUi
from QtCore import *
from QtGui import *

#from python_qt_binding import loadUi
#from python_qt_binding import QtGui
#from python_qt_binding import QtCore
#from python_qt_binding.QtGui import *
#from python_qt_binding.QtCore import *

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
        #self.rqt_ui.orientWidget.show()
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
        
        self.cam_xpos = 0;
        self.cam_ypos = 0;
        self.cam_zpos = 0; 
        
        self.createActions()
        self.createMenus()
        self.createButtons()
        self.createScrollbars()
        
        # IMU UI Dialog
        self.imuWidget = imu_dialog()
        
        # Main UI
        centralWidget = QtGui.QWidget()
        self.setCentralWidget(centralWidget)

		# 3D Orientation Widget
        self.glWidget = GLWidget()

        self.glWidgetArea = QtGui.QScrollArea()
        self.glWidgetArea.setWidget(self.glWidget)
        self.glWidgetArea.setWidgetResizable(True)
        self.glWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.glWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        self.glWidgetArea.setMinimumSize(50, 50)
        
        xSlider = self.createSlider(self.glWidget.xRotationChanged, self.glWidget.setXRotation)
        ySlider = self.createSlider(self.glWidget.yRotationChanged, self.glWidget.setYRotation)
        zSlider = self.createSlider(self.glWidget.zRotationChanged, self.glWidget.setZRotation)
        
        # Orientation Widget
        self.orientWidget = OrientWidget()
        #style = QtCore.Qt.BrushStyle(QtCore.Qt.Dense1Pattern)
        #self.orientWidget.setBrush(QtGui.QBrush(QtCore.Qt.gray, style))

        self.orientWidgetArea = QtGui.QScrollArea()
        self.orientWidgetArea.setWidget(self.orientWidget)
        self.orientWidgetArea.setWidgetResizable(True)
        self.orientWidgetArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.orientWidgetArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.orientWidgetArea.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        self.orientWidgetArea.setMinimumSize(50, 50)
        
        # Camera Widget
        
        centralLayout = QtGui.QGridLayout()
        centralLayout.addWidget(self.glWidgetArea, 0, 0)
        centralLayout.addWidget(self.orientWidgetArea, 0, 1)
        centralLayout.addWidget(xSlider, 1, 0, 1, 2)
        centralLayout.addWidget(ySlider, 2, 0, 1, 2)
        centralLayout.addWidget(zSlider, 3, 0, 1, 2)
        centralLayout.addWidget(self.pb_imu)
        centralWidget.setLayout(centralLayout)
        
        xSlider.setValue(15 * 16)
        ySlider.setValue(345 * 16)
        zSlider.setValue(0 * 16)

        self.setWindowTitle("Arthrobot Main Window")
        self.resize(1000, 500)

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

    def createSlider(self, changedSignal, setterSlot):
        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, 360 * 16)
        slider.setSingleStep(16)
        slider.setPageStep(15 * 16)
        slider.setTickInterval(15 * 16)
        slider.setTickPosition(QtGui.QSlider.TicksRight)
        slider.valueChanged.connect(setterSlot)
        changedSignal.connect(slider.setValue)
        return slider

    def createButtons(self):
        self.pb_imu = QtGui.QPushButton("IMU Data")
        self.pb_cam_up = QtGui.QPushButton("Up")
        self.pb_cam_down = QtGui.QPushButton("Down")
        self.pb_cam_left = QtGui.QPushButton("Left")
        self.pb_cam_right = QtGui.QPushButton("Right")

        self.pb_imu.clicked[bool].connect(self._handle_pb_imu_clicked)
        self.pb_cam_up.clicked[bool].connect(self._handle_pb_cam_down_clicked)
        self.pb_cam_down.clicked[bool].connect(self._handle_pb_imu_clicked)
        self.pb_cam_left.clicked[bool].connect(self._handle_pb_cam_left_clicked)
        self.pb_cam_right.clicked[bool].connect(self._handle_pb_cam_right_clicked)

    def createScrollbars(self):
        scrollbar_cam_zoom = QtGui.QScrollBar()
        scrollbar_cam_zoom.setOrientation(QtCore.Qt.Horizontal)
        scrollbar_cam_zoom.setMinimum(0)
        scrollbar_cam_zoom.setMaximum(100)
        scrollbar_cam_zoom.connect(scrollbar_cam_zoom, QtCore.SIGNAL("sliderMoved(int)"), self._cam_slider_moved)

    def _cam_slider_moved(self, value):
        self.cam_zpos = value;
        self.mainWindow.tb_cam_zpos.setText(str(self.cam_zpos));

    def _handle_pb_cam_up_clicked(self):
        if (self.cam_ypos <= 50):
            self.cam_ypos += 1;
        else:
            self.cam_ypos = 50;
        self.mainWindow.tb_cam_ypos.setText(str(self.cam_ypos));
    
    def _handle_pb_cam_down_clicked(self):
        if (self.cam_ypos >= -50):
            self.cam_ypos -= 1;
        else:
            self.cam_ypos = -50;
        self.mainWindow.tb_cam_ypos.setText(str(self.cam_ypos));
        
    def _handle_pb_cam_left_clicked(self):
        if (self.cam_xpos >= -50):
            self.cam_xpos -= 1;
        else:
            self.cam_xpos = -50;
        self.mainWindow.tb_cam_xpos.setText(str(self.cam_xpos));

    def _handle_pb_cam_right_clicked(self):
        if (self.cam_xpos <= 50):
            self.cam_xpos += 1;
        else:
            self.cam_xpos = 50;
        self.mainWindow.tb_cam_xpos.setText(str(self.cam_xpos));

    def _handle_pb_imu_clicked(self):
        self.imuWidget._dialog.show()
        rospy.Subscriber("/imu/data", Imu, self.imuWidget.updateImu)
