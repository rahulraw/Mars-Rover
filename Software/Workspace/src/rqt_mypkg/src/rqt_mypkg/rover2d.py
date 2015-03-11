from PyQt4 import QtCore, QtGui
from QtCore import *
from QtGui import *

import math

class OrientWidget(QtGui.QWidget):
    
    def __init__(self, parent=None):
        super(OrientWidget, self).__init__(parent)

        self.setFixedSize(300,300)
        self.size = self.size()
        self.width = self.size.width()
        self.height = self.size.height()

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
        
        self.incline = 0.0
        self.topLeftWheelAngle = 45.0
        self.topRightWheelAngle = 0.0
        self.botLeftWheelAngle = 0.0
        self.botRightWheelAngle = 0.0

    def setBrush(self, brush):
        self.brush = brush
        self.update()
        
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setPen(Qt.black)
        painter.setBrush(self.brush)
        
        self.updateTopOrientation(painter)
        # self.drawRover2DTop(painter)
        #painter.rotate(45)
        # painter.translate(25.0,10.0)
        # painter.setBrush(QtGui.QBrush(QtCore.Qt.red, self.style))
        # painter.drawRoundedRect(65,60,25,50,5.0,5.0)
        # painter.drawRoundedRect(160,60,25,50,5.0,5.0)
        # painter.drawRoundedRect(65,210,25,50,5.0,5.0)
        # painter.drawRoundedRect(160,210,25,50,5.0,5.0)
        #painter.rotate(-45)
        # painter.translate(-25.0,-10.0)
        # self.drawRover2DSide(painter)  

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

    def drawWheelOrientation(self, p, wheel, angle):
        p.setPen(Qt.black)
        p.setBrush(QtGui.QBrush(QtCore.Qt.red, self.style))
        # p.rotate(angle)
        # # p.translate((wheel.x()-wheel.x()*math.cos(angle)), (wheel.x()-wheel.x()*math.sin(angle)))
        # p.drawRoundedRect(wheel.x() - self.wheelWidth/2, wheel.y() - self.wheelHeight/2, self.wheelWidth, self.wheelHeight, self.wheelRadius, self.wheelRadius)
        # # p.translate(-(wheel.x()-wheel.x()*math.cos(angle)), -(wheel.x()-wheel.x()*math.sin(angle)))
        # p.rotate(-angle)
        transform = QtGui.QTransform()
        # transform.translate((35+self.wheelWidth)/2, (35+self.wheelHeight)/2)
        transform.rotate(angle)

        p.setTransform(transform)
        p.drawRoundedRect(wheel.x(), wheel.y(),self.wheelWidth, self.wheelHeight, self.wheelRadius, self.wheelRadius)

    def drawRover2DSide(self,p):
        p.setPen(Qt.black)
        p.setBrush(QtGui.QBrush(QtCore.Qt.gray, self.style)) 
        p.drawRect(self.sideView)
        p.setBrush(QtGui.QBrush(QtCore.Qt.red, self.style))
        # p.drawEllipse(250,150, 50, 50)
        # p.drawEllipse(400,150, 50, 50)
        p.drawText(QPoint(50,290),"Rover Incline: "+str(self.incline) + " deg")

