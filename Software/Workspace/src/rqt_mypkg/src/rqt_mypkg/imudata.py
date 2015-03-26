import os
import rospy
import rospkg
import tf, math

from PyQt4 import QtCore, QtGui
from PyQt4.uic import loadUi
from PyQt4.QtCore import * 
from PyQt4.QtGui import *
from sensor_msgs.msg import Imu

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

class imuWidget:

    def __init__(self):
        rp = rospkg.RosPack()
        imu_file = os.path.join(rp.get_path('rqt_mypkg'), 'src/rqt_mypkg/resource', 'imu_widget.ui')
        self._widget = loadUi(imu_file)

        self.imuT = imuThread()

    def startImu(self):
        self.imuT.running = True
        self._widget.connect(self.imuT, QtCore.SIGNAL('updateImu(PyQt_PyObject)'), self.updateImu, Qt.QueuedConnection)
        self.imuT.start()

    def getEuler(self, data):
        quaternion = (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0] * 180 / math.pi + 180
        self.pitch = euler[1] * 180 / math.pi + 180
        self.yaw = euler[2] * 180 / math.pi + 180

    def updateImu(self, data):

        self.getEuler(data)

        self._widget.lineEdit_Quat_x.setText(QString(data.orientation.x))
        self._widget.lineEdit_Quat_y.setText(QString(data.orientation.y))
        self._widget.lineEdit_Quat_z.setText(QString(data.orientation.z))
        self._widget.lineEdit_Quat_w.setText(QString(data.orientation.w))
        self._widget.lineEdit_AV_x.setText(QString(data.angular_velocity.x))
        self._widget.lineEdit_AV_y.setText(QString(data.angular_velocity.y))
        self._widget.lineEdit_AV_z.setText(QString(data.angular_velocity.z))
        self._widget.lineEdit_LA_x.setText(QString(data.linear_acceleration.x))
        self._widget.lineEdit_LA_y.setText(QString(data.linear_acceleration.y))
        self._widget.lineEdit_LA_z.setText(QString(data.linear_acceleration.z))

        self._widget.lineEdit_Euler_roll.setText(QString(self.roll))
        self._widget.lineEdit_Euler_pitch.setText(QString(self.pitch))
        self._widget.lineEdit_Euler_yaw.setText(QString(self.yaw))


