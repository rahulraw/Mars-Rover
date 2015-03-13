import math, numpy
import rospy
import sys

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from steering.msg import RoverInfo

class OrientWidget(QtGui.QWidget):
    
    def __init__(self, parent=None):
        super(OrientWidget, self).__init__(parent)

        rospy.Subscriber("RoverInfo", RoverInfo, self._wheel_slider_moved)

        self.createScrollbars()

        self.incline = 0.0
        self.topLeftWheelAngle = 25.0
        self.topRightWheelAngle = 45.0
        self.botLeftWheelAngle = 60.0
        self.botRightWheelAngle = 90.0
        self.mode = "default"

        self.topViewX = 35
        self.topViewY = 35
        self.topViewWidgth = 50
        self.topViewHeight = 75
        self.topLeftWheel = QPoint(self.topViewX, self.topViewY)
        self.topRightWheel = QPoint(self.topViewX+self.topViewWidgth, self.topViewY)
        self.botLeftWheel = QPoint(self.topViewX, self.topViewY+self.topViewHeight)
        self.botRightWheel = QPoint(self.topViewX+self.topViewWidgth, self.topViewY+self.topViewHeight)
        self.wheelWidth = 20
        self.wheelHeight = 30
        self.wheelRadius = 7.0

        self.sideViewX = 115
        self.sideViewY = 200
        self.sideViewWidgth = 75
        self.sideViewHeight = 35

        self.topView = QtCore.QRect(self.topViewX,self.topViewY,self.topViewWidgth,self.topViewHeight)
        self.sideView = QtCore.QRect(self.sideViewX,self.sideViewY,self.sideViewWidgth,self.sideViewHeight)
        
        self.brush = QtGui.QBrush()
        self.setBrush(QtCore.Qt.NoBrush)
        
        self.style = QtCore.Qt.BrushStyle(QtCore.Qt.Dense1Pattern)
        
        self.setBackgroundRole(QtGui.QPalette.Base)
        self.setAutoFillBackground(True)

        centralLayout = QtGui.QFormLayout()
        # centralLayout.addWidget(self.scrollbar_wheel)
        self.setLayout(centralLayout)

        self.setFixedSize(300,300)
        self.size = self.size()
        self.width = self.size.width()
        self.height = self.size.height()

    def setBrush(self, brush):
        self.brush = brush
        self.update()
        
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setPen(Qt.black)
        painter.setBrush(self.brush)
        
        self.updateTopOrientation(painter)

    def updateTopOrientation(self, p):
        self.drawRover2DTop(p)
        self.drawWheelOrientation(p, self.topLeftWheel, self.topLeftWheelAngle)
        self.drawWheelOrientation(p, self.topRightWheel, self.topRightWheelAngle)
        self.drawWheelOrientation(p, self.botLeftWheel, self.botLeftWheelAngle)
        self.drawWheelOrientation(p, self.botRightWheel, self.botRightWheelAngle)

    def drawRover2DTop(self, p):
        p.setPen(Qt.black)
        p.setBrush(QtGui.QBrush(QtCore.Qt.gray, self.style)) 
        p.drawRect(self.topView)
        p.drawText(QPoint(self.topViewX + 100,self.topViewY),"Top Left: "+str(self.topLeftWheelAngle) + " deg")
        p.drawText(QPoint(self.topViewX + 100,self.topViewY + 25),"Top Right: "+str(self.topRightWheelAngle) + " deg")
        p.drawText(QPoint(self.topViewX + 100,self.topViewY + 50),"Bottom Left: "+str(self.botLeftWheelAngle) + " deg")
        p.drawText(QPoint(self.topViewX + 100,self.topViewY + 75),"Bottom Right: "+str(self.botRightWheelAngle) + " deg")
        p.drawText(QPoint(self.topViewX + 100,self.topViewY + 100),"Mode: "+ self.mode)

    def drawWheelOrientation(self, p, wheel, angle):
        p.setPen(Qt.black)
        p.setBrush(QtGui.QBrush(QtCore.Qt.red, self.style))

        theta = -angle*math.pi/180
        coord = numpy.array([wheel.x(),wheel.y()])
        rotMatrix = numpy.array([[numpy.cos(theta), -numpy.sin(theta)], 
                         [numpy.sin(theta),  numpy.cos(theta)]])
        coord2 = numpy.dot(rotMatrix,coord)
        dx = coord2[0] - coord[0]
        dy = coord2[1] - coord[1]
        p.rotate(angle)
        p.translate(dx,dy)
        p.drawRoundedRect(coord[0] - self.wheelWidth/2, coord[1] - self.wheelHeight/2, self.wheelWidth, self.wheelHeight, self.wheelRadius, self.wheelRadius)
        p.translate(-dx,-dy)
        p.rotate(-angle)

    def drawRover2DSide(self,p):
        p.setPen(Qt.black)
        p.setBrush(QtGui.QBrush(QtCore.Qt.gray, self.style)) 
        p.drawRect(self.sideView)
        p.setBrush(QtGui.QBrush(QtCore.Qt.red, self.style))
        # p.drawEllipse(250,150, 50, 50)
        # p.drawEllipse(400,150, 50, 50)
        p.drawText(QPoint(50,290),"Rover Incline: "+str(self.incline) + " deg")

    def createScrollbars(self):
        self.scrollbar_wheel = QtGui.QScrollBar()
        self.scrollbar_wheel.setOrientation(QtCore.Qt.Horizontal)
        self.scrollbar_wheel.setMinimum(0)
        self.scrollbar_wheel.setMaximum(360)
        self.scrollbar_wheel.connect(self.scrollbar_wheel, QtCore.SIGNAL("sliderMoved(int)"), self._wheel_slider_moved)

    def _wheel_slider_moved(self, value):
        self.topLeftWheelAngle = float(value.front_left_angle)
        self.topRightWheelAngle = float(value.front_right_angle)
        self.botLeftWheelAngle = float(value.back_left_angle)
        self.botRightWheelAngle = float(value.back_right_angle)
        self.mode = value.mode
        self.update()

    
