#Embedded file name: /home/jerbotron/Documents/uwrobotics.uwmrt/Software/Workspace/src/rqt_mypkg/src/rqt_mypkg/rover3d.py
import sys, math, numpy, tf
import rospy, rospkg
from PyQt4 import QtCore, QtGui, QtOpenGL
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from OpenGL import GL
from sensor_msgs.msg import Imu
from rover2d import OrientWidget
try:
    from PyQt4.QtCore import QString
except ImportError:
    QString = str

class imuThread(QtCore.QThread):

    def __init__(self):
        QtCore.QThread.__init__(self)

    def run(self):
        rospy.Subscriber('/imu/data', Imu, self.callback)

    def callback(self, data):
        self.emit(QtCore.SIGNAL('updateImu(PyQt_PyObject)'), data)


class Window(QtGui.QWidget):

    def __init__(self):
        super(Window, self).__init__()
        self.glWidget = GLWidget()
        mainLayout = QtGui.QVBoxLayout()
        mainLayout.addWidget(self.glWidget)
        self.setLayout(mainLayout)
        self.setWindowTitle('Hello GL')


class GLWidget(QtOpenGL.QGLWidget):

    def __init__(self, rover2dWidget = None, parent = None):
        super(GLWidget, self).__init__(parent)

        self.r2d_widget = OrientWidget(rover2dWidget)

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
        self.trolltechGreen = QtGui.QColor.fromCmykF(0.4, 0.0, 1.0, 0.0)
        self.trolltechPurple = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)
        self.scrubjerryred = QtGui.QColor.fromCmykF(0, 1.0, 1.0, 0.0)
        self.kevinblue = QtGui.QColor.fromCmykF(1.0, 1.0, 0.0, 0.0)
        self.archieyellow = QtGui.QColor.fromCmykF(0.0, 0.0, 1.0, 0.0)
        self.petercyan = QtGui.QColor.fromCmykF(1.0, 0.0, 0.0, 0.0)
        self.magenta = QtGui.QColor.fromCmykF(0.0, 1.0, 0.0, 0.0)

        self.setFixedSize(300,300)
        self.startImu()

    def startImu(self):
        self.imuT = imuThread()
        self.connect(self.imuT, QtCore.SIGNAL('updateImu(PyQt_PyObject)'), self.updateImu, Qt.QueuedConnection)
        self.imuT.start()

    def calibrateImu(self):
        self.roll_offset = -self.roll
        self.pitch_offset = -self.pitch
        self.yaw_offset = -self.yaw
        # print 'roll = ' + str(int(self.roll)) + ' | pitch = ' + str(int(self.pitch)) + ' | yaw = ' + str(int(self.yaw))
        # print 'roll offset = ' + str(int(self.roll_offset)) + ' | pitch offset = ' + str(int(self.pitch_offset)) + ' | yaw offset = ' + str(int(self.yaw_offset))

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 5760

        while angle > 5760:
            angle -= 5760

        return angle

    def updateImu(self, data):
        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.prev_yaw = self.yaw
        quaternion = (data.orientation.x,
         data.orientation.y,
         data.orientation.z,
         data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0] * 180 / math.pi + 180 + self.roll_offset
        self.pitch = euler[1] * 180 / math.pi + 180 + self.pitch_offset
        self.yaw = euler[2] * 180 / math.pi + 180 + self.yaw_offset

        if not math.isnan(self.roll) and not math.isnan(self.prev_roll):
            if int(math.floor(self.roll)) != int(math.floor(self.prev_roll)):
                self.setXRotation(self.roll * 16)
                self.r2d_widget.setIncline(self.roll)
                self.r2d_widget.update()

        if not math.isnan(self.pitch) and not math.isnan(self.prev_pitch):
            if int(math.floor(self.pitch)) != int(math.floor(self.prev_pitch)):
                self.setZRotation(-self.pitch * 16)

        if not math.isnan(self.yaw) and not math.isnan(self.prev_yaw):
            if int(math.floor(self.yaw)) != int(math.floor(self.prev_yaw)):
                self.setYRotation(self.yaw * 16)


    def minimumSizeHint(self):
        return QtCore.QSize(50, 50)

    def sizeHint(self):
        return QtCore.QSize(400, 400)

    def setXRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.xRot:
            self.xRot = angle
            self.updateGL()

    def setYRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.yRot:
            self.yRot = angle
            self.updateGL()

    def setZRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.zRot:
            self.zRot = angle
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
        self.qglColor(self.trolltechGreen)
        GL.glVertex3d(x1, y1, -d)
        GL.glVertex3d(x2, y2, -d)
        GL.glVertex3d(x3, y3, -d)
        GL.glVertex3d(x4, y4, -d)
        self.qglColor(self.scrubjerryred)
        GL.glVertex3d(x4, y4, d)
        GL.glVertex3d(x3, y3, d)
        GL.glVertex3d(x2, y2, d)
        GL.glVertex3d(x1, y1, d)
        self.qglColor(self.kevinblue)
        GL.glVertex3d(x1, y1, d)
        GL.glVertex3d(x2, y2, d)
        GL.glVertex3d(x2, y2, -d)
        GL.glVertex3d(x1, y1, -d)
        self.qglColor(self.archieyellow)
        GL.glVertex3d(x1, y1, d)
        GL.glVertex3d(x1, y1, -d)
        GL.glVertex3d(x4, y4, -d)
        GL.glVertex3d(x4, y4, d)
        self.qglColor(self.petercyan)
        GL.glVertex3d(x2, y2, d)
        GL.glVertex3d(x3, y3, d)
        GL.glVertex3d(x3, y3, -d)
        GL.glVertex3d(x2, y2, -d)
        self.qglColor(self.magenta)
        GL.glVertex3d(x3, y3, d)
        GL.glVertex3d(x4, y4, d)
        GL.glVertex3d(x4, y4, -d)
        GL.glVertex3d(x3, y3, -d)
        self.drawWheel(x3, y3, d)
        self.drawWheel(x3, y3, -d)
        self.drawWheel(x4 - 0.06, y4, d)
        self.drawWheel(x4 - 0.06, y4, -d)
        GL.glEnd()
        GL.glEndList()
        return genList

    def drawWheel(self, x, y, d):
        wl = 0.06
        ww = 0.04
        wd = 0.04
        x1 = x
        x2 = x + wl
        x3 = x + wl
        x4 = x
        y1 = y - ww
        y2 = y - ww
        y3 = y + ww
        y4 = y + ww
        z1 = d - wd
        z2 = d + wd
        self.qglColor(QtGui.QColor.fromCmykF(0.0, 0.0, 0.0, 1.0))
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
