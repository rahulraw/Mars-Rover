from PyQt4 import QtCore, QtGui
from QtCore import *
from QtGui import *

class OrientWidget(QtGui.QWidget):
    
    def __init__(self, parent=None):
        super(OrientWidget, self).__init__(parent)
        self.size = self.size()
        
        self.brush = QtGui.QBrush()
        self.setBrush(QtCore.Qt.NoBrush)
        
        self.style = QtCore.Qt.BrushStyle(QtCore.Qt.Dense1Pattern)
        
        self.setBackgroundRole(QtGui.QPalette.Base)
        self.setAutoFillBackground(True)
        
        self.incline = 0.0

    def setBrush(self, brush):
        self.brush = brush
        self.update()
        
    def paintEvent(self, event):
        self.topView = QtCore.QRect(75,85,100,150)
        self.sideView = QtCore.QRect(275,100,150,75)

        painter = QtGui.QPainter(self)
        painter.setPen(Qt.black)
        painter.setBrush(self.brush)
      
        self.drawRover2DTop(painter)
        painter.rotate(45)
        painter.translate(25.0,10.0)
        painter.setBrush(QtGui.QBrush(QtCore.Qt.red, self.style))
        painter.drawRoundedRect(65,60,25,50,5.0,5.0)
        painter.drawRoundedRect(160,60,25,50,5.0,5.0)
        painter.drawRoundedRect(65,210,25,50,5.0,5.0)
        painter.drawRoundedRect(160,210,25,50,5.0,5.0)
        painter.rotate(-45)
        painter.translate(-25.0,-10.0)
        self.drawRover2DSide(painter)  
        painter.drawText(QPoint(280,300),"Rover Incline: "+str(self.incline) + " degrees")
        

    def drawRover2DTop(self, p):
        p.setPen(Qt.black)
        p.setBrush(QtGui.QBrush(QtCore.Qt.gray, self.style)) 
        p.drawRect(self.topView)

    def drawRover2DSide(self,p):
        p.setPen(Qt.black)
        p.setBrush(QtGui.QBrush(QtCore.Qt.gray, self.style)) 
        p.drawRect(self.sideView)
        p.setBrush(QtGui.QBrush(QtCore.Qt.red, self.style))
        p.drawEllipse(250,150, 50, 50)
        p.drawEllipse(400,150, 50, 50)

