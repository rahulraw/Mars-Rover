#!/usr/bin/env python
import sys, math, numpy, tf
import rospy, rospkg

from PyQt4 import QtCore, QtGui, QtOpenGL
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from OpenGL import GL

from sensor_msgs.msg import Imu

try:  
    from PyQt4.QtCore import QString  
except ImportError:  
    # we are using Python3 so QString is not defined  
    QString = str 

class imuThread(QtCore.QThread):
    def __init__(self):
        QtCore.QThread.__init__(self)

    def run(self):
        print("imu thread run")
        rospy.Subscriber("/imu/data", Imu, self.callback)
        
    def callback(self, data):
        self.emit(QtCore.SIGNAL('updateImu(PyQt_PyObject)'), data)

class Window(QtGui.QWidget):
    def __init__(self):
        super(Window, self).__init__()

        self.glWidget = GLWidget()

        # self.xSlider = self.createSlider()
        # self.ySlider = self.createSlider()
        # self.zSlider = self.createSlider()

        # self.xSlider.valueChanged.connect(self.glWidget.setXRotation)
        # self.glWidget.xRotationChanged.connect(self.xSlider.setValue)
        # self.ySlider.valueChanged.connect(self.glWidget.setYRotation)
        # self.glWidget.yRotationChanged.connect(self.ySlider.setValue)
        # self.zSlider.valueChanged.connect(self.glWidget.setZRotation)
        # self.glWidget.zRotationChanged.connect(self.zSlider.setValue)

        mainLayout = QtGui.QVBoxLayout()
        mainLayout.addWidget(self.glWidget)
        # mainLayout.addWidget(self.xSlider)
        # mainLayout.addWidget(self.ySlider)
        # mainLayout.addWidget(self.zSlider)
        self.setLayout(mainLayout)

        # self.xSlider.setValue(15 * 16)
        # self.ySlider.setValue(345 * 16)
        # self.zSlider.setValue(0 * 16)

        self.setWindowTitle("Hello GL")

    # def createSlider(self):
    #     slider = QtGui.QSlider(QtCore.Qt.Vertical)

    #     slider.setRange(0, 360 * 16)
    #     slider.setSingleStep(16)
    #     slider.setPageStep(15 * 16)
    #     slider.setTickInterval(15 * 16)
    #     slider.setTickPosition(QtGui.QSlider.TicksRight)

    #     return slider

class GLWidget(QtOpenGL.QGLWidget):
    # xRotationChanged = QtCore.pyqtSignal(float)
    # yRotationChanged = QtCore.pyqtSignal(float)
    # zRotationChanged = QtCore.pyqtSignal(float)

    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)

        self.object = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0

        self.roll = 0.0
        self.prev_roll = 0.0
        self.roll_offset = 0.0

        self.pitch = 0.0
        self.prev_pitch = 0.0
        self.pitch_offset = 0.0

        self.yaw = 0.0
        self.prev_yaw = 0.0
        self.yaw_offset = 0.0

        self.lastPos = QtCore.QPoint()

        self.trolltechGreen = QtGui.QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)
        self.scrubjerryred = QtGui.QColor.fromCmykF(0, 1.0, 1.0, 0.0)
        self.kevinblue = QtGui.QColor.fromCmykF(1.0 , 1.0, 0.0, 0.0)
        self.archieyellow = QtGui.QColor.fromCmykF(0.0, 0.0, 1.0, 0.0)
        self.petercyan = QtGui.QColor.fromCmykF(1.0, 0.0, 0.0, 0.0)
        self.magenta = QtGui.QColor.fromCmykF(0.0, 1.0, 0.0, 0.0)

        # self.setXRotation(45*16)

        self.startImu()

    def startImu(self):
        self.imuT = imuThread()
        self.connect(self.imuT, QtCore.SIGNAL('updateImu(PyQt_PyObject)'), self.updateImu, Qt.QueuedConnection)
        self.imuT.start()

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

    def updateImu(self, data):
        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.prev_yaw = self.yaw

        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0] * 180 / math.pi + 180 + self.roll_offset
        self.pitch = euler[1] * 180 / math.pi + 180 + self.pitch_offset
        self.yaw = euler[2] * 180 / math.pi + 180 + self.yaw_offset

        # print("roll = " + str(self.roll) + " | pitch = " + str(self.pitch) + " | yaw = " + str(self.yaw))

        if (int(self.roll) != int(self.prev_roll)):
            self.setXRotation(self.roll * 16)
        if (int(self.pitch) != int(self.prev_pitch)):
            self.setYRotation(self.pitch * 16)
        if (int(self.yaw) != int(self.prev_yaw)):
            self.setZRotation(self.yaw * 16)

    def minimumSizeHint(self):
        return QtCore.QSize(50, 50)

    def sizeHint(self):
        return QtCore.QSize(400, 400)

    def setXRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.xRot:
            self.xRot = angle
            # self.xRotationChanged.emit(angle)
            self.updateGL()

    def setYRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.yRot:
            self.yRot = angle
            # self.yRotationChanged.emit(angle)
            self.updateGL()

    def setZRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.zRot:
            self.zRot = angle
            # self.zRotationChanged.emit(angle)
            self.updateGL()

    def initializeGL(self):
        self.qglClearColor(self.trolltechPurple.dark())
        self.object = self.makeObject()
        GL.glShadeModel(GL.GL_FLAT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_CULL_FACE)

    def paintGL(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glLoadIdentity()
        GL.glTranslated(0.0, 0.0, -10.0)
        GL.glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)
        GL.glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)
        GL.glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)
        GL.glCallList(self.object)

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        GL.glViewport((width - side) // 2, (height - side) // 2, side, side)

        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)

    # def mousePressEvent(self, event):
    #     self.lastPos = event.pos()

    # def mouseMoveEvent(self, event):
    #     dx = event.x() - self.lastPos.x()
    #     dy = event.y() - self.lastPos.y()

    #     if event.buttons() & QtCore.Qt.LeftButton:
    #         self.setXRotation(self.xRot + 8 * dy)
    #         self.setYRotation(self.yRot + 8 * dx)
    #     elif event.buttons() & QtCore.Qt.RightButton:
    #         self.setXRotation(self.xRot + 8 * dy)
    #         self.setZRotation(self.zRot + 8 * dx)

    #     self.lastPos = event.pos()

    def makeObject(self):
        genList = GL.glGenLists(1)
        GL.glNewList(genList, GL.GL_COMPILE)

        GL.glBegin(GL.GL_QUADS)

        l = 0.2
        w = 0.1
        d = 0.2

        x1 = -l
        y1 = -w
        x2 = l
        y2 = -w
        x3 = l
        y3 = w
        x4 = -l
        y4 = w

        self.qglColor(self.trolltechGreen) #back
        GL.glVertex3d(x1, y1, -d)
        GL.glVertex3d(x2, y2, -d)
        GL.glVertex3d(x3, y3, -d)
        GL.glVertex3d(x4, y4, -d)

        self.qglColor(self.scrubjerryred) #front
        GL.glVertex3d(x4, y4, d)
        GL.glVertex3d(x3, y3, d)
        GL.glVertex3d(x2, y2, d)
        GL.glVertex3d(x1, y1, d)

        self.qglColor(self.kevinblue) #top
        GL.glVertex3d(x1, y1, d)
        GL.glVertex3d(x2, y2, d)
        GL.glVertex3d(x2, y2, -d)
        GL.glVertex3d(x1, y1, -d)

        self.qglColor(self.archieyellow) #leftside
        GL.glVertex3d(x1, y1, d)
        GL.glVertex3d(x1, y1, -d)
        GL.glVertex3d(x4, y4, -d)
        GL.glVertex3d(x4, y4, d)

        self.qglColor(self.petercyan) #rightside
        GL.glVertex3d(x2, y2, d)
        GL.glVertex3d(x3, y3, d)
        GL.glVertex3d(x3, y3, -d)
        GL.glVertex3d(x2, y2, -d)

        self.qglColor(self.magenta) #bottom
        GL.glVertex3d(x3, y3, d)
        GL.glVertex3d(x4, y4, d)
        GL.glVertex3d(x4, y4, -d)
        GL.glVertex3d(x3, y3, -d)



        #draw wheels
        #front right wheel: point 3 +d
        self.drawWheel(x3,y3,d)
        #back right: p3 -d
        self.drawWheel(x3,y3,-d)
        #front left: p4 +d
        self.drawWheel(x4-0.06,y4,d)
        #back left: p4 -d
        self.drawWheel(x4-0.06,y4,-d)

        GL.glEnd()
        GL.glEndList()

        return genList

    def drawWheel(self,x,y,d):
        wl = 0.06
        ww = 0.04
        wd = 0.04

        x1 = x
        x2 = x+wl
        x3 = x+wl
        x4 = x

        y1 = y-ww
        y2 = y-ww
        y3 = y+ww
        y4 = y+ww

        z1 = d-wd
        z2 = d+wd

        self.qglColor(QtGui.QColor.fromCmykF(0.0, 0.0, 0.0, 1.0)) #back
        GL.glVertex3d(x1, y1, z1)
        GL.glVertex3d(x2, y2, z1)
        GL.glVertex3d(x3, y3, z1)
        GL.glVertex3d(x4, y4, z1)

        GL.glVertex3d(x4, y4, z2)
        GL.glVertex3d(x3, y3, z2)
        GL.glVertex3d(x2, y2, z2)
        GL.glVertex3d(x1, y1, z2)

        GL.glVertex3d(x1, y1, z2)
        GL.glVertex3d(x2, y2, z2)
        GL.glVertex3d(x2, y2, z1)
        GL.glVertex3d(x1, y1, z1)

        GL.glVertex3d(x1, y1, z2)
        GL.glVertex3d(x1, y1, z1)
        GL.glVertex3d(x4, y4, z1)
        GL.glVertex3d(x4, y4, z2)

        GL.glVertex3d(x2, y2, z2)
        GL.glVertex3d(x3, y3, z2)
        GL.glVertex3d(x3, y3, z1)
        GL.glVertex3d(x2, y2, z1)

        GL.glVertex3d(x3, y3, z2)
        GL.glVertex3d(x4, y4, z2)
        GL.glVertex3d(x4, y4, z1)
        GL.glVertex3d(x3, y3, z1)
    # def makeObject(self):
    #     genList = GL.glGenLists(1)
    #     GL.glNewList(genList, GL.GL_COMPILE)

    #     GL.glBegin(GL.GL_QUADS)

    #     x1 = +0.06
    #     y1 = -0.14
    #     x2 = +0.14
    #     y2 = -0.06
    #     x3 = +0.08
    #     y3 = +0.00
    #     x4 = +0.30
    #     y4 = +0.22

    #     self.quad(x1, y1, x2, y2, y2, x2, y1, x1)
    #     self.quad(x3, y3, x4, y4, y4, x4, y3, x3)

    #     self.extrude(x1, y1, x2, y2)
    #     self.extrude(x2, y2, y2, x2)
    #     self.extrude(y2, x2, y1, x1)
    #     self.extrude(y1, x1, x1, y1)
    #     self.extrude(x3, y3, x4, y4)
    #     self.extrude(x4, y4, y4, x4)
    #     self.extrude(y4, x4, y3, x3)

    #     NumSectors = 200

    #     for i in range(NumSectors):
    #         angle1 = (i * 2 * math.pi) / NumSectors
    #         x5 = 0.30 * math.sin(angle1)
    #         y5 = 0.30 * math.cos(angle1)
    #         x6 = 0.20 * math.sin(angle1)
    #         y6 = 0.20 * math.cos(angle1)

    #         angle2 = ((i + 1) * 2 * math.pi) / NumSectors
    #         x7 = 0.20 * math.sin(angle2)
    #         y7 = 0.20 * math.cos(angle2)
    #         x8 = 0.30 * math.sin(angle2)
    #         y8 = 0.30 * math.cos(angle2)

    #         self.quad(x5, y5, x6, y6, x7, y7, x8, y8)

    #         self.extrude(x6, y6, x7, y7)
    #         self.extrude(x8, y8, x5, y5)

    #     GL.glEnd()
    #     GL.glEndList()

    #     return genList

    # def quad(self, x1, y1, x2, y2, x3, y3, x4, y4):
    #     self.qglColor(self.trolltechGreen)

    #     GL.glVertex3d(x1, y1, -0.05)
    #     GL.glVertex3d(x2, y2, -0.05)
    #     GL.glVertex3d(x3, y3, -0.05)
    #     GL.glVertex3d(x4, y4, -0.05)

    #     GL.glVertex3d(x4, y4, +0.05)
    #     GL.glVertex3d(x3, y3, +0.05)
    #     GL.glVertex3d(x2, y2, +0.05)
    #     GL.glVertex3d(x1, y1, +0.05)

    # def extrude(self, x1, y1, x2, y2):
    #     self.qglColor(self.trolltechGreen.dark(250 + int(100 * x1)))

    #     GL.glVertex3d(x1, y1, +0.05)
    #     GL.glVertex3d(x2, y2, +0.05)
    #     GL.glVertex3d(x2, y2, -0.05)
    #     GL.glVertex3d(x1, y1, -0.05)